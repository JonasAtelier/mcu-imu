#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "imu_priv.h"

#if CONFIG_IMU_WT901

#include "imu_log.h"
#include "wt901.h"
#include "wt901_proto.h"

/* Two output periods at the module's default 10 Hz (§6.2, p.35). */
#define DEFAULT_TIMEOUT_MS	200

/*
 * The most bytes one read looks through before giving up. Two full cycles
 * with every one of the eleven frame types on (§6.1) is 242; a start in the
 * middle of a frame costs at most ten more.
 */
#define SCAN_BUDGET		256

#define TYPE_BIT(type)		(1u << ((type) - WT901_TYPE_FIRST))

struct wt901 {
	struct imu_bus bus;
};

/*
 * A walk along the stream. The window holds the bytes not yet ruled out as
 * the start of a frame, so a header hiding inside a rejected one is still
 * found and no byte is dropped unexamined.
 */
struct wt901_scan {
	struct imu_bus *bus;
	uint8_t window[WT901_FRAME_LEN];
	size_t have;		/* bytes in window */
	size_t budget;		/* bytes still allowed to arrive */
	size_t received;	/* bytes that did arrive */
	bool drop_stale;	/* the first receive starts on fresh data */
	unsigned int valid;	/* frames whose SUM matched */
};

static const struct imu_ops wt901_ops;

/* Our state, or NULL if this imu_dev was not probed by this driver. */
static struct wt901 *to_wt901(struct imu_dev *imu)
{
	if (!imu || imu->ops != &wt901_ops)
		return NULL;
	return imu->priv;
}

static void wt901_scan_start(struct wt901_scan *scan, struct imu_bus *bus)
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
static int wt901_next_frame(struct wt901_scan *scan,
			    uint8_t frame[WT901_FRAME_LEN])
{
	size_t need, next;
	int ret;

	for (;;) {
		need = WT901_FRAME_LEN - scan->have;
		if (need > scan->budget)
			return scan->valid ? -ENODATA : -EBADMSG;

		ret = imu_bus_receive(scan->bus, scan->window + scan->have,
				      need, scan->drop_stale);
		scan->drop_stale = false;
		if (ret)
			return ret;
		scan->budget -= need;
		scan->received += need;
		scan->have = WT901_FRAME_LEN;

		if (wt901_frame_valid(scan->window)) {
			memcpy(frame, scan->window, WT901_FRAME_LEN);
			scan->have = 0;
			scan->valid++;
			return 0;
		}

		/* Not a frame from here: slide to the next 0x55, if any. */
		for (next = 1; next < WT901_FRAME_LEN; next++)
			if (scan->window[next] == WT901_FRAME_HEADER)
				break;
		scan->have = WT901_FRAME_LEN - next;
		memmove(scan->window, scan->window + next, scan->have);
	}
}

/*
 * The next frame of each type in want, a mask of TYPE_BIT()s, from fresh
 * data. frames[] is indexed by type - 0x50; frames of other types pass by.
 */
static int wt901_collect(struct wt901_scan *scan, unsigned int want,
			 uint8_t frames[][WT901_FRAME_LEN])
{
	uint8_t frame[WT901_FRAME_LEN];
	unsigned int got = 0;
	int ret;

	while ((got & want) != want) {
		ret = wt901_next_frame(scan, frame);
		if (ret)
			return ret;
		if (want & TYPE_BIT(frame[1]) & ~got) {
			memcpy(frames[frame[1] - WT901_TYPE_FIRST], frame,
			       WT901_FRAME_LEN);
			got |= TYPE_BIT(frame[1]);
		}
	}
	return 0;
}

/* -------------------------------------------------------------- read ----- */

#define ACCEL_INDEX	(WT901_TYPE_ACCEL - WT901_TYPE_FIRST)
#define GYRO_INDEX	(WT901_TYPE_GYRO - WT901_TYPE_FIRST)
#define ANGLE_INDEX	(WT901_TYPE_ANGLE - WT901_TYPE_FIRST)

/*
 * imu_ops.read: the next accelerometer and gyroscope frames after fresh
 * data starts, in whichever order they come (§6.1 gives none). They are
 * consecutive frames, not one instant. Temperature is the accelerometer
 * frame's (§6.1.2, p.31).
 */
static int wt901_read(struct imu_dev *imu, struct imu_sample *s)
{
	struct wt901 *d = imu->priv;
	uint8_t frames[WT901_TYPE_LAST - WT901_TYPE_FIRST + 1][WT901_FRAME_LEN];
	struct wt901_scan scan;
	const uint8_t *a, *g;
	int ret;

	wt901_scan_start(&scan, &d->bus);
	ret = wt901_collect(&scan, TYPE_BIT(WT901_TYPE_ACCEL) |
				   TYPE_BIT(WT901_TYPE_GYRO), frames);
	if (ret)
		return ret;

	a = frames[ACCEL_INDEX] + WT901_FRAME_DATA;
	g = frames[GYRO_INDEX] + WT901_FRAME_DATA;
	for (int i = 0; i < 3; i++) {
		s->accel[i] = wt901_scaled(wt901_value(a + 2 * i),
					   WT901_ACCEL_FULL_SCALE_G) * IMU_G;
		s->gyro[i] = wt901_scaled(wt901_value(g + 2 * i),
					  WT901_GYRO_FULL_SCALE_DPS) *
			     IMU_DEG_TO_RAD;
	}
	s->temp_c = (float)wt901_value(a + 6) / WT901_TEMP_LSB_PER_C;
	return 0;
}

static const struct imu_ops wt901_ops = {
	.read = wt901_read,
};

/* ----------------------------------------------------------- extras ------ */

int wt901_read_angles(struct imu_dev *imu, float rpy_deg[3])
{
	struct wt901 *d = to_wt901(imu);
	uint8_t frames[WT901_TYPE_LAST - WT901_TYPE_FIRST + 1][WT901_FRAME_LEN];
	struct wt901_scan scan;
	const uint8_t *angle;
	int ret;

	if (!d || !rpy_deg)
		return -EINVAL;

	wt901_scan_start(&scan, &d->bus);
	ret = wt901_collect(&scan, TYPE_BIT(WT901_TYPE_ANGLE), frames);
	if (ret)
		return ret;

	/* §6.1.4, p.32: roll (x), pitch (y), yaw (z). */
	angle = frames[ANGLE_INDEX] + WT901_FRAME_DATA;
	for (int i = 0; i < 3; i++)
		rpy_deg[i] = wt901_scaled(wt901_value(angle + 2 * i),
					  WT901_ANGLE_FULL_SCALE_DEG);
	return 0;
}

int wt901_command(struct imu_dev *imu, uint8_t address, uint16_t value)
{
	struct wt901 *d = to_wt901(imu);
	/* §6.2, p.35: 0xFF 0xAA address dataL dataH. */
	const uint8_t tx[4] = {
		WT901_COMMAND_2, address, (uint8_t)value, (uint8_t)(value >> 8),
	};

	if (!d)
		return -EINVAL;
	return d->bus.ops->transfer(&d->bus, WT901_COMMAND_1, tx, sizeof(tx),
				    NULL, 0);
}

/* ------------------------------------------------------- probe/remove ---- */

int wt901_probe(struct imu_dev *imu,
#ifdef ESP_PLATFORM
		uart_port_t uart,
#else
		UART_HandleTypeDef *uart,
#endif
		const struct wt901_config *cfg)
{
	uint8_t frames[WT901_TYPE_LAST - WT901_TYPE_FIRST + 1][WT901_FRAME_LEN];
	struct wt901_scan scan;
	struct imu_bus bus;
	struct wt901 *d;
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

	imu->name = "wt901";

	timeout_ms = cfg && cfg->timeout_ms ? cfg->timeout_ms :
		     DEFAULT_TIMEOUT_MS;
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
	wt901_scan_start(&scan, &bus);
	ret = wt901_collect(&scan, TYPE_BIT(WT901_TYPE_ACCEL) |
				   TYPE_BIT(WT901_TYPE_GYRO), frames);
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

	d = calloc(1, sizeof(*d));
	if (!d) {
		imu_bus_detach(&bus);
		return -ENOMEM;
	}
	d->bus = bus;

	imu->ops = &wt901_ops;
	imu->priv = d;
	imu_info(imu, "streaming on UART, %u valid frames seen",
		 scan.valid);
	return 0;
}

void wt901_remove(struct imu_dev *imu)
{
	struct wt901 *d = to_wt901(imu);

	if (!d)
		return;

	imu_bus_detach(&d->bus);
	free(d);
	imu->ops = NULL;
	imu->priv = NULL;
}

#endif /* CONFIG_IMU_WT901 */
