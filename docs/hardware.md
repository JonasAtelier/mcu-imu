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
