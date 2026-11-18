# Hardware wiring

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Pins

On ESP-IDF any GPIO works; the examples in `examples/esp/` set theirs at the
top of `main.c`. On STM32, pins are the ones the examples in `examples/stm/`
use, on an F4: I2C1 on PB8/PB9 (AF4), SPI1 on
PA5/PA6/PA7 (AF5), chip selects on PD14/PD15. Any pin your chip routes to the
same peripheral works; check the alternate-function table in its datasheet.
The bus init — pins, clocks, speed, SPI mode — is the app's; each example
shows one.

### MPU-6050 — `examples/*/mpu6050`

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | the bare chip is 3.3 V only. A GY-521 has a regulator and takes 5 V too |
| VLOGIC | 3.3 V | the separate I/O supply; tie it to VDD unless you are level shifting |
| GND | GND | shared ground, always |
| SDA | PB9 | I2C1_SDA, AF4, open drain |
| SCL | PB8 | I2C1_SCL, AF4, open drain |
| AD0 | GND | low is 0x68, high is 0x69 (`.address = 0x69`). Those are the only two, so two chips per bus |
| INT | not used by this driver; leave open | `mpu6050_data_ready()` reads `INT_STATUS` over I2C instead |
| AUX_DA, AUX_CL | a magnetometer, or open | the auxiliary bus — see [The auxiliary bus](chips.md#the-auxiliary-bus) |

A GY-521 silkscreens AUX_DA and AUX_CL as XDA and XCL and labels VDD as VCC.
Both it and the Adafruit board pictured in the [chip reference](chips.md#mpu-6050) regulate their
input, so they take 3.3 V or 5 V where the bare chip does not.

### ICM-45686 — `examples/*/icm45686`

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | TDK rates the part for 1.71–3.6 V. No datasheet ships here, so check your breakout |
| VDDIO | 3.3 V | the interface supply, and it sets the logic level. Match the STM32 |
| GND | GND | shared ground, always |
| SCLK | PA5 | SPI1_SCK, AF5 |
| SDI | PA7 | SPI1_MOSI, AF5 |
| SDO | PA6 | SPI1_MISO, AF5 |
| CS | PD14 | a plain GPIO output, idle high; one per chip on the bus |
| INT1, INT2 | nothing | not used by this driver; `icm45686_data_ready()` reads `INT1_STATUS0` over SPI instead. Leave open |

SPI mode 3, 8-bit frames, at most 24 MHz. No I2C or I3C rows: this driver is
SPI only, whatever the part can do. Breakouts label `SDO` as **AD0**, because
it doubles as the I2C address bit on parts wired for it.

### BMI088 — `examples/*/bmi088` (I2C)

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | analog and digital domain, 2.4–3.6 V |
| VDDIO | 3.3 V | interface supply, 1.2–3.6 V, and it sets the logic level |
| GNDA, GNDIO | GND | shared ground, always |
| PS | VDDIO | protocol select for the gyroscope: high is I2C. Sampled at power-on, not later |
| SCK / SCL | PB8 | I2C1_SCL, AF4, open drain |
| SDI / SDA | PB9 | I2C1_SDA, AF4, open drain |
| SDO1 | GND | accelerometer address: GND is 0x18, VDDIO is 0x19 |
| SDO2 | GND | gyroscope address: GND is 0x68, VDDIO is 0x69. **An MPU-6050 at 0x68 on the same bus collides with this** |
| CSB1 | VDDIO or open | accelerometer chip select. A rising edge switches it to SPI until power reset, so leave it alone on I2C |
| CSB2 | open | gyroscope chip select |
| INT1–INT4 | nothing | not used by this driver; leave open. `bmi088_data_ready()` reads both status registers instead |

### BMI088 — `examples/*/bmi088_spi` (SPI)

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | analog and digital domain, 2.4–3.6 V |
| VDDIO | 3.3 V | interface supply, 1.2–3.6 V, and it sets the logic level |
| GNDA, GNDIO | GND | shared ground, always |
| PS | GND | protocol select for the gyroscope: low is SPI. Sampled at power-on, not later |
| SCK / SCL | PA5 | SPI1_SCK, AF5 |
| SDI / SDA | PA7 | SPI1_MOSI, shared by both dies |
| SDO1 | PA6 | SPI1_MISO, accelerometer |
| SDO2 | PA6 | gyroscope MISO, same wire as SDO1 |
| CSB1 | PD14, with a pull-up | accelerometer chip select. It starts in I2C mode; the probe's throwaway read gives it the rising edge that switches it to SPI until power reset |
| CSB2 | PD15, with a pull-up | gyroscope chip select. Both pull-ups keep the dies deselected while the STM32 boots |
| INT1–INT4 | nothing | not used by this driver; leave open |

SPI mode 0 (the example) or 3, 8-bit frames, at most 10 MHz.

Names are from table 14 of the [Bosch BMI088 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf) (BST-BMI088-DS001). The
MPU-6050 and ICM-45686 names are the chips' usual ones; neither datasheet
ships in this repo.

### ICM-42688-P — `examples/esp/icm42688` (SPI)

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | 1.71–3.6 V |
| VDDIO | 3.3 V | 1.71–3.6 V, and it sets the logic level |
| GND | GND | shared ground; pin 7 (RESV) to GND too |
| AP_CS | GPIO 10 | chip select |
| AP_SCLK | GPIO 12 | SPI clock |
| AP_SDI | GPIO 11 | MOSI |
| AP_SDO | GPIO 13 | MISO |
| INT1 | a GPIO, or nothing | data ready, if `data_ready_int1.enable` |
| INT2 / FSYNC / CLKIN | GND | FSYNC unused: "Connect to GND if FSYNC not used" |
| RESV (2, 3, 10, 11) | nothing or GND | |

SPI mode 0 or 3, 8-bit frames, at most 24 MHz; the example runs at 8 MHz.

### ICM-42688-P — `examples/stm/icm42688` (I2C)

| pin | goes to | note |
|---|---|---|
| VDD, VDDIO | 3.3 V | 1.71–3.6 V each |
| GND | GND | pin 7 (RESV) to GND too |
| AP_CS | VDDIO | high: "Connect to VDDIO if using AP I3CSM / I2C interface" (Table 10) |
| AP_SCL | PB8 | I2C1_SCL, AF4, open drain |
| AP_SDA | PB9 | I2C1_SDA, AF4, open drain. **External pull-ups on SDA and SCL** |
| AP_AD0 | GND | address: GND is 0x68, VDDIO is 0x69. **An MPU-6050, BMI270 or BMI088 gyroscope at 0x68 on the same bus collides with this** |
| INT1 | an EXTI pin, or nothing | data ready on INT1, as the example configures it: push-pull, active high, pulsed |
| INT2 / FSYNC / CLKIN | GND | FSYNC unused |

I2C at up to 1 MHz. Names, pins and ranges are from the ICM-42688-P
datasheet (DS-000347 rev. 1.6): Table 10 and Figure 5 for the pins, Table 3
for the supplies, 9.3 and 9.6 for the interfaces.

### BMI270 — `examples/esp/bmi270` (I2C)

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | core supply, 1.71–3.6 V |
| VDDIO | 3.3 V | interface supply, 1.2–3.6 V, and it sets the logic level |
| GND, GNDIO | GND | shared ground, always |
| CSB | VDDIO | stays high: the interface is I2C after power-up and turns SPI on the first CSB rising edge |
| SCX | SCL | GPIO 22 in the example |
| SDX | SDA | GPIO 21 in the example. **External pull-ups on SDA and SCL**; the example only turns on the ESP32's weak ones |
| SDO | GND | address: GND is 0x68, VDDIO is 0x69. **An MPU-6050, or a BMI088 gyroscope, at 0x68 on the same bus collides with this** |
| INT1 | a GPIO, or nothing | data ready, if `data_ready_int1.enable`; else leave open |
| INT2, ASDx, ASCx, OCSB, OSDO | nothing | INT2, the auxiliary and the OIS interface: not used by this driver; leave open |

I2C at up to 1 MHz (Fast-mode Plus).

### BMI270 — `examples/stm/bmi270` (SPI)

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | core supply, 1.71–3.6 V |
| VDDIO | 3.3 V | interface supply, 1.2–3.6 V |
| GND, GNDIO | GND | shared ground |
| CSB | PD14, with a pull-up | chip select. The interface starts as I2C after power-up and after every soft reset; the probe's throwaway reads give it the rising edges that switch it to SPI |
| SCX | PA5 | SPI1_SCK, AF5 |
| SDX | PA7 | SPI1_MOSI |
| SDO | PA6 | SPI1_MISO |
| INT1 | an EXTI pin, or nothing | data ready on INT1, as the example configures it: push-pull, active high |
| INT2, ASDx, ASCx, OCSB, OSDO | nothing | leave open |

SPI mode 0 (the example) or 3, 8-bit frames, at most 10 MHz.

Names and ranges are from the BMI270 datasheet (BST-BMI270-DS000-08, rev.
1.6): section 7.1 and Table 17 for the pins, Table 1 for the supplies, 6.3
to 6.5 for the interfaces.

### WT901C-TTL — no example

Pin names are the WT901 core's, from §3 of `docs/WT901.pdf` (p.5); the cased
C-TTL is assumed to bring out the same signals. Only the four serial lines
are used: this driver does not support the IIC mode or the D0–D3 ports.

| pin | goes to | note |
|---|---|---|
| VCC | 3.3 V or 5 V | the module regulates it: 3.3–5 V (§1, p.4; §2 item 1) |
| GND | GND | shared ground, always |
| TX | the MCU's UART RX | the module's serial output, TTL level (§3) |
| RX | the MCU's UART TX | the module's serial input, TTL level (§3) |

TX and RX cross over. The link is TTL level, never RS232, at 9600 bps with
no parity out of the box (chapter 6, p.30); this library reads "stop bit and
parity bit 0" there as one stop bit and 8 data bits. The UART setup is the
app's: an installed driver on ESP-IDF, a `HAL_UART_Init()`ed handle on the
HAL.

### JY901 — `examples/esp/jy901` (UART)

Pin names are from §3 of `docs/WT901.pdf` (p.5), which names the module
JY-901 in its IIC sections. On the UART only the four serial lines are used;
SCL, SDA and D0–D3 stay open.

| pin | goes to | note |
|---|---|---|
| VCC | 3.3 V or 5 V | the module regulates it: 3.3–5 V (§2 item 1, p.4) |
| GND | GND | shared ground, always |
| TX | the MCU's UART RX | GPIO 18 in the example; the module's serial output, TTL level (§3) |
| RX | the MCU's UART TX | GPIO 17 in the example; the module's serial input, TTL level (§3) |

TX and RX cross over. The link is TTL level, never RS232, at 9600 bps, 8
data bits, no parity, one stop bit out of the box (chapter 6, p.30), as for
the WT901C-TTL.

### JY901 — `examples/stm/jy901` (I2C)

| pin | goes to | note |
|---|---|---|
| VCC | 3.3 V or 5 V | 3.3–5 V (§2 item 1, p.4) |
| GND | GND | shared ground, always |
| SCL | PB8 | I2C1_SCL, AF4, open drain |
| SDA | PB9 | I2C1_SDA, AF4, open drain. **4.7 k pull-ups on SDA and SCL**: the module's bus is open drain (§5.2, p.26) |
| TX, RX, D0–D3 | nothing | leave open |

I2C at up to 400 kHz (chapter 7, p.46), at 7-bit address 0x50 unless it was
changed (§6.2.32, p.45). **An EEPROM at 0x50 on the same bus collides with
this**; the probe refuses one, but both cannot share the address.

### LIS2DH12 — no example

The bare chip on a board, I2C. Pin names are the datasheet's (section 4,
"Pin description"; UNVERIFIED, see the [chip reference](chips.md#lis2dh12)).

| pin | goes to | note |
|---|---|---|
| VDD, VDD_IO | 3.3 V | 1.71–3.6 V; 100 nF at VDD |
| GND, RES | GND | RES is a reserved pin tied to GND |
| CS | VDD_IO | high selects I2C; low would select SPI |
| SCL/SPC | SCL | 4.7 k pull-up to VDD_IO |
| SDA/SDI/SDO | SDA | 4.7 k pull-up to VDD_IO |
| SDO/SA0 | GND or VDD_IO | address bit 0: 0x18 low, 0x19 high. Tied to a rail, `sa0_pullup_off` saves the internal pull-up's current |
| INT1 | a GPIO | push-pull, no pull-up needed; polarity by `int_active_low` |
| INT2 | nothing | not used by the driver |

I2C at up to 400 kHz. The INT1 pin drives both levels from power-up, at
active high until the probe says otherwise, so it idles low before the
probe and high after it with `int_active_low`.
