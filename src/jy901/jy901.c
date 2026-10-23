#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "imu_priv.h"

#if CONFIG_IMU_JY901

#include "imu_log.h"
#include "jy901.h"
#include "jy901_regs.h"

/* Two output periods at the module's default 10 Hz (§6.2, p.35). */
#define UART_TIMEOUT_MS		200
#define I2C_TIMEOUT_MS		100

/*
 * The most bytes one stream read looks through before giving up. Two full
 * cycles with every one of the eleven frame types on (§6.1) is 242; a start
 * in the middle of a frame costs at most ten more.
 */
#define SCAN_BUDGET		256

#define TYPE_BIT(type)		(1u << ((type) - JY901_TYPE_FIRST))
#define TYPE_COUNT		(JY901_TYPE_LAST - JY901_TYPE_FIRST + 1)
#define ACCEL_INDEX		(JY901_TYPE_ACCEL - JY901_TYPE_FIRST)
#define GYRO_INDEX		(JY901_TYPE_GYRO - JY901_TYPE_FIRST)
#define ANGLE_INDEX		(JY901_TYPE_ANGLE - JY901_TYPE_FIRST)

struct jy901 {
	struct imu_bus bus;	/* a UART or an I2C device */
};

/*
 * A walk along the UART stream. The window holds the bytes not yet ruled
 * out as the start of a frame, so a header hiding inside a rejected one is
 * still found and no byte is dropped unexamined.
 */
struct jy901_scan {
	struct imu_bus *bus;
	uint8_t window[JY901_FRAME_LEN];
	size_t have;		/* bytes in window */
	size_t budget;		/* bytes still allowed to arrive */
	size_t received;	/* bytes that did arrive */
	bool drop_stale;	/* the first receive starts on fresh data */
	unsigned int valid;	/* frames whose SUM matched */
};

static const struct imu_ops jy901_ops;

/* Our state, or NULL if this imu_dev was not probed by this driver. */
static struct jy901 *to_jy901(struct imu_dev *imu)
{
	if (!imu || imu->ops != &jy901_ops)
		return NULL;
	return imu->priv;
}

static bool on_uart(const struct jy901 *d)
{
	return d->bus.ops->kind == IMU_BUS_UART;
}

/* ------------------------------------------------------------ stream ----- */

static void jy901_scan_start(struct jy901_scan *scan, struct imu_bus *bus)
{
	memset(scan, 0, sizeof(*scan));
	scan->bus = bus;
	scan->budget = SCAN_BUDGET;
	scan->drop_stale = true;
}

/*
 * The next frame whose header, type and SUM all check out. Out of budget:
 * -ENODATA if some frames were valid, -EBADMSG if none ever was. A stream
 * that stops is the bus's -ETIMEDOUT.
 */
static int jy901_next_frame(struct jy901_scan *scan,
			    uint8_t frame[JY901_FRAME_LEN])
{
	size_t need, next;
	int ret;

	for (;;) {
		need = JY901_FRAME_LEN - scan->have;
		if (need > scan->budget)
			return scan->valid ? -ENODATA : -EBADMSG;

		ret = imu_bus_receive(scan->bus, scan->window + scan->have,
				      need, scan->drop_stale);
		scan->drop_stale = false;
		if (ret)
			return ret;
		scan->budget -= need;
		scan->received += need;
		scan->have = JY901_FRAME_LEN;

		if (jy901_frame_valid(scan->window)) {
			memcpy(frame, scan->window, JY901_FRAME_LEN);
			scan->have = 0;
			scan->valid++;
			return 0;
		}

		/* Not a frame from here: slide to the next 0x55, if any. */
		for (next = 1; next < JY901_FRAME_LEN; next++)
			if (scan->window[next] == JY901_FRAME_HEADER)
				break;
		scan->have = JY901_FRAME_LEN - next;
		memmove(scan->window, scan->window + next, scan->have);
	}
}

/*
 * The next frame of each type in want, a mask of TYPE_BIT()s, from fresh
 * data. frames[] is indexed by type - 0x50; frames of other types pass by.
 */
static int jy901_collect(struct jy901_scan *scan, unsigned int want,
			 uint8_t frames[][JY901_FRAME_LEN])
{
	uint8_t frame[JY901_FRAME_LEN];
	unsigned int got = 0;
	int ret;

	while ((got & want) != want) {
		ret = jy901_next_frame(scan, frame);
		if (ret)
			return ret;
		if (want & TYPE_BIT(frame[1]) & ~got) {
			memcpy(frames[frame[1] - JY901_TYPE_FIRST], frame,
			       JY901_FRAME_LEN);
			got |= TYPE_BIT(frame[1]);
		}
	}
	return 0;
}

/* -------------------------------------------------------------- read ----- */

/*
 * Three accelerometer values, three gyroscope values and the temperature,
 * each low byte first, as both a frame and the registers hold them.
 */
static void jy901_fill_sample(struct imu_sample *s, const uint8_t *accel,
			      const uint8_t *gyro, const uint8_t *temp)
{
	for (int i = 0; i < 3; i++) {
		s->accel[i] = jy901_scaled(jy901_value(accel + 2 * i),
					   JY901_ACCEL_FULL_SCALE_G) * IMU_G;
		s->gyro[i] = jy901_scaled(jy901_value(gyro + 2 * i),
					  JY901_GYRO_FULL_SCALE_DPS) *
			     IMU_DEG_TO_RAD;
	}
	s->temp_c = (float)jy901_value(temp) / JY901_TEMP_LSB_PER_C;
}

/*
 * UART: the next accelerometer and gyroscope frames after fresh data
 * starts, in whichever order they come (§6.1 gives none). They are
 * consecutive frames, not one instant. Temperature is the accelerometer
 * frame's (§6.1.2, p.31).
 */
static int jy901_read_uart(struct imu_bus *bus, struct imu_sample *s)
{
	uint8_t frames[TYPE_COUNT][JY901_FRAME_LEN];
	struct jy901_scan scan;
	const uint8_t *accel;
	int ret;

	jy901_scan_start(&scan, bus);
	ret = jy901_collect(&scan, TYPE_BIT(JY901_TYPE_ACCEL) |
				   TYPE_BIT(JY901_TYPE_GYRO), frames);
	if (ret)
		return ret;

	accel = frames[ACCEL_INDEX] + JY901_FRAME_DATA;
	jy901_fill_sample(s, accel, frames[GYRO_INDEX] + JY901_FRAME_DATA,
			  accel + 6);
	return 0;
}

/* I2C: AX to TEMP in one read (§7.2, p.49), so one moment. */
static int jy901_read_i2c(struct imu_bus *bus, struct imu_sample *s)
{
	uint8_t burst[JY901_BURST_LEN];
	int ret;

	ret = imu_reg_read(bus, JY901_BURST_FIRST, burst, sizeof(burst));
	if (ret)
		return ret;

	jy901_fill_sample(s, burst + JY901_BURST_ACCEL * JY901_REG_BYTES,
			  burst + JY901_BURST_GYRO * JY901_REG_BYTES,
			  burst + JY901_BURST_TEMP * JY901_REG_BYTES);
	return 0;
}

/* imu_ops.read */
static int jy901_read(struct imu_dev *imu, struct imu_sample *s)
{
	struct jy901 *d = imu->priv;

	if (on_uart(d))
		return jy901_read_uart(&d->bus, s);
	return jy901_read_i2c(&d->bus, s);
}

static const struct imu_ops jy901_ops = {
	.read = jy901_read,
};

/* ----------------------------------------------------------- extras ------ */

int jy901_read_angles(struct imu_dev *imu, float roll_pitch_yaw_deg[3])
{
	struct jy901 *d = to_jy901(imu);
	uint8_t frames[TYPE_COUNT][JY901_FRAME_LEN];
	uint8_t registers[3 * JY901_REG_BYTES];
	struct jy901_scan scan;
	const uint8_t *angle;
	int ret;

	if (!d || !roll_pitch_yaw_deg)
		return -EINVAL;

	if (on_uart(d)) {
		jy901_scan_start(&scan, &d->bus);
		ret = jy901_collect(&scan, TYPE_BIT(JY901_TYPE_ANGLE), frames);
		angle = frames[ANGLE_INDEX] + JY901_FRAME_DATA;
	} else {
		/* §7.2, p.49: Roll, Pitch, Yaw from 0x3d, six bytes. */
		ret = imu_reg_read(&d->bus, JY901_REG_ROLL, registers,
				   sizeof(registers));
		angle = registers;
	}
	if (ret)
		return ret;

	/* §6.1.4, p.32: roll (x), pitch (y), yaw (z). */
	for (int i = 0; i < 3; i++)
		roll_pitch_yaw_deg[i] =
			jy901_scaled(jy901_value(angle + 2 * i),
				     JY901_ANGLE_FULL_SCALE_DEG);
	return 0;
}

int jy901_command(struct imu_dev *imu, uint8_t address, uint16_t value)
{
	struct jy901 *d = to_jy901(imu);
	/* §6.2 reminder 2, p.35: 0xFF 0xAA address dataL dataH. */
	const uint8_t serial[4] = {
		JY901_COMMAND_2, address, (uint8_t)value, (uint8_t)(value >> 8),
	};

	if (!d)
		return -EINVAL;
	if (on_uart(d))
		return d->bus.ops->transfer(&d->bus, JY901_COMMAND_1, serial,
					    sizeof(serial), NULL, 0);
	/* §7.1, p.48: the register address, then dataL dataH. */
	return d->bus.ops->transfer(&d->bus, address, serial + 2, 2, NULL, 0);
}

/* ------------------------------------------------------- probe/remove ---- */

/* Move a checked bus into fresh state and bind it; -ENOMEM detaches it. */
static int jy901_bind(struct imu_dev *imu, struct imu_bus *bus)
{
	struct jy901 *d = calloc(1, sizeof(*d));

	if (!d) {
		imu_bus_detach(bus);
		return -ENOMEM;
	}
	d->bus = *bus;
	imu->ops = &jy901_ops;
	imu->priv = d;
	return 0;
}

#if defined(ESP_PLATFORM) || defined(HAL_UART_MODULE_ENABLED)
int jy901_probe_uart(struct imu_dev *imu,
#ifdef ESP_PLATFORM
		     uart_port_t uart,
#else
		     UART_HandleTypeDef *uart,
#endif
		     const struct jy901_uart_config *config)
{
	uint8_t frames[TYPE_COUNT][JY901_FRAME_LEN];
	struct jy901_scan scan;
	struct imu_bus bus;
	uint32_t timeout_ms;
	int ret;

	if (!imu)
		return -EINVAL;
#ifndef ESP_PLATFORM
	if (!uart)
		return -EINVAL;
#endif
	/* Bound already: a second probe would leak the first binding. */
	if (imu->ops || imu->priv)
		return -EBUSY;

	imu->name = "jy901";

	timeout_ms = config && config->timeout_ms ? config->timeout_ms :
		     UART_TIMEOUT_MS;
	ret = imu_uart_attach(&bus, uart, timeout_ms);
	if (ret) {
		imu_err(imu, "cannot use the UART (%d)", ret);
		return ret;
	}

	/*
	 * Anything there at all? Decided on a bus on the stack, before
	 * anything is allocated. Silence is -ENXIO, at debug level like a
	 * missing I2C ACK.
	 */
	jy901_scan_start(&scan, &bus);
	ret = jy901_collect(&scan, TYPE_BIT(JY901_TYPE_ACCEL) |
				   TYPE_BIT(JY901_TYPE_GYRO), frames);
	if (ret == -ETIMEDOUT && !scan.received) {
		imu_dbg(imu, "nothing streaming on the UART (%d)", ret);
		ret = -ENXIO;
	} else if (ret == -EBADMSG) {
		imu_err(imu, "bytes arrive, but no frame's SUM matches: "
			"check the baud rate");
	} else if (ret == -ENODATA) {
		imu_err(imu, "frames arrive, but not both 0x51 and 0x52: turn "
			"their output on (return content, register 0x02)");
	} else if (ret) {
		imu_err(imu, "reading the stream failed (%d)", ret);
	}
	if (ret) {
		imu_bus_detach(&bus);
		return ret;
	}

	ret = jy901_bind(imu, &bus);
	if (ret)
		return ret;
	imu_info(imu, "streaming on UART, %u valid frames seen",
		 scan.valid);
	return 0;
}
#endif

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
int jy901_probe_i2c(struct imu_dev *imu,
		    const struct jy901_i2c_config *bus_config)
{
	uint8_t iicaddr[JY901_REG_BYTES];
	uint8_t burst[JY901_BURST_LEN];
	struct imu_bus bus;
	uint8_t address;
	uint32_t timeout_ms;
	uint32_t scl_hz = 0;
	int ret;

	if (!imu || !bus_config || !bus_config->bus)
		return -EINVAL;
	if (imu->ops || imu->priv)
		return -EBUSY;

	imu->name = "jy901";

	address = bus_config->address ? bus_config->address :
		  JY901_I2C_ADDRESS;
	if (address > JY901_I2C_ADDRESS_MAX) {
		imu_err(imu, "address 0x%02x is not 7-bit", address);
		return -EINVAL;
	}
#ifdef ESP_PLATFORM
	if (bus_config->clock_hz > JY901_I2C_MAX_HZ) {
		imu_err(imu, "I2C clock %u Hz is above 400 kHz",
			(unsigned int)bus_config->clock_hz);
		return -EINVAL;
	}
	scl_hz = bus_config->clock_hz ? bus_config->clock_hz :
		 JY901_I2C_MAX_HZ;
#endif
	timeout_ms = bus_config->timeout_ms > 0 ?
		     (uint32_t)bus_config->timeout_ms : I2C_TIMEOUT_MS;

	/* There at all? See imu_i2c_probe(); decided before allocating. */
	ret = imu_i2c_probe(bus_config->bus, address, timeout_ms);
	if (ret) {
		imu_err(imu, "nothing answering at 0x%02x (%d)", address, ret);
		return ret;
	}
	ret = imu_i2c_attach(&bus, bus_config->bus, address, scl_hz,
			     timeout_ms);
	if (ret) {
		imu_err(imu, "cannot add the I2C device (%d)", ret);
		return ret;
	}

	/*
	 * The module has no identity register. IICADDR holds the address it
	 * answers at, low byte first with a zero high byte (§6.2.32, p.45),
	 * so reading it back tells the module from another part at 0x50.
	 */
	ret = imu_reg_read(&bus, JY901_REG_IICADDR, iicaddr, sizeof(iicaddr));
	if (!ret && (uint16_t)jy901_value(iicaddr) != address) {
		imu_err(imu, "IICADDR reads 0x%04x at 0x%02x: not a JY901",
			(unsigned int)(uint16_t)jy901_value(iicaddr), address);
		ret = -ENODEV;
	} else if (!ret) {
		ret = imu_reg_read(&bus, JY901_BURST_FIRST, burst,
				   sizeof(burst));
	}
	if (ret) {
		if (ret != -ENODEV)
			imu_err(imu, "reading the registers failed (%d)", ret);
		imu_bus_detach(&bus);
		return ret;
	}

	ret = jy901_bind(imu, &bus);
	if (ret)
		return ret;
	imu_info(imu, "up on I2C at 0x%02x", address);
	return 0;
}
#endif

void jy901_remove(struct imu_dev *imu)
{
	struct jy901 *d = to_jy901(imu);

	if (!d)
		return;

	imu_bus_detach(&d->bus);
	free(d);
	imu->ops = NULL;
	imu->priv = NULL;
}

#endif /* CONFIG_IMU_JY901 */
