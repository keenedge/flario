# airio v0: parts list and XIAO nRF52840 pin map

airio is a small BLE-only vario, separate from the ESP32-S3 Flario Rev A in
[parts-list.md](parts-list.md). v0 is a carrier PCB with a Seeed XIAO nRF52840
soldered on as a castellated module, plus a baro, an IMU and a piezo beeper.
Firmware is Zephyr (nRF Connect SDK) in [`airio/firmware`](../airio/firmware).

Decided 2026-10-24: XIAO nRF52840, BMP581, LSM6DSV16X.

## Parts

Prices are LCSC, converted at 1.40 CAD per USD (2026-10-23 close), excluding
shipping and duty. "Live" rows were checked against LCSC's product API on
2026-10-24; "page" rows come from LCSC product page snapshots and should be
rechecked before ordering. "From" is LCSC's lowest tier, so qty 1 costs more.

| Ref | Function | Part | LCSC # | CAD each | LCSC stock | Source | Notes |
|---|---|---|---|---|---|---|---|
| U1 | MCU + BLE + USB + charger | Seeed XIAO nRF52840 (102010448) | [C17209540](https://www.lcsc.com/product-detail/C17209540.html) | from C$14.82 | 158 | page | Castellated module. Onboard BQ25101 charger, 3.3 V LDO, USB-C, UF2 bootloader. Non-Sense version (no onboard IMU). |
| U2 | Barometer | Bosch BMP581 | [C5362283](https://www.lcsc.com/product-detail/C5362283.html) | C$3.59 | **0** | live | LGA-10 2x2 mm. I2C. Out of stock at LCSC: order elsewhere or use U2-alt. |
| U2-alt | Barometer (footprint compatible) | Bosch BMP580 | [C22391138](https://www.lcsc.com/product-detail/C22391138.html) | C$1.70 | 14,944 | live | Same package and pinout. Confirm the Zephyr `bmp581` driver accepts its chip ID before relying on it. |
| U3 | 6-axis IMU | ST LSM6DSV16XTR | [C5267406](https://www.lcsc.com/product-detail/C5267406.html) | C$4.75 | 6,395 | live | LGA-14 2.5x3 mm. I2C. On-chip sensor fusion (SFLP). |
| BZ1 | Piezo beeper | Murata PKLCS1212E4001-R1 | [C113159](https://www.lcsc.com/product-detail/C113159.html) | from C$0.92 | 7,334 | page | 12x12 mm SMD, 4 kHz resonance, 84 dB. Driven differentially from two GPIOs (about 6.6 Vpp) so no driver stage is needed. |
| C1-C4 | Sensor decoupling, 100 nF 0603 X7R | Samsung CL10B104KB8NNNC | [C1591](https://www.lcsc.com/product-detail/C1591.html) | from C$0.01 | conflicting on page | page | One each on BMP581 VDD, BMP581 VDDIO, LSM6DSV16X VDD, LSM6DSV16X VDDIO. JLC basic part. |
| R1, R2 | I2C pull-ups, 4.7 kΩ 0603 | UNI-ROYAL 0603WAF4701T5E | [C23162](https://www.lcsc.com/product-detail/C23162.html) | from C$0.002 | 4,646,600 | page | SDA and SCL to 3V3. |
| R3 | BMP581 INT pull-up, 10 kΩ 0603 | TBD | TBD | | | | Only if INT is configured open-drain; firmware uses push-pull, so this is optional. |
| SW1 | User / power button | TBD | TBD | | | | Tactile switch to GND, internal pull-up. |
| BT1 | 1S LiPo, about 150 mAh | Not on LCSC | | | | | Solder to the XIAO's BAT+/BAT- pads. Size from the 8 h power estimate with 2x margin. |

## Pin map

All signals use the XIAO's 3.3 V rail. The 1.8 V rail idea from the power
discussion doesn't apply to v0 because the XIAO has a fixed 3.3 V regulator;
revisit it for a bare-chip v1.

| XIAO pin | nRF52840 | airio signal | Direction | Notes |
|---|---|---|---|---|
| D0 | P0.02 | IMU_INT1 | in | LSM6DSV16X INT1, data-ready, active high. |
| D1 | P0.03 | BARO_INT | in | BMP581 INT, data-ready, push-pull active high. |
| D2 | P0.28 | BUZZ_A | out | PWM0 channel 0. |
| D3 | P0.29 | BUZZ_B | out | PWM0 channel 1, inverted, so the piezo sees twice the swing. |
| D4 | P0.04 | I2C_SDA | io | i2c0. BMP581 at 0x47 (SDO high), LSM6DSV16X at 0x6A (SDO low). |
| D5 | P0.05 | I2C_SCL | out | i2c0, 400 kHz. |
| D6 | P1.11 | BTN | in | SW1 to GND, internal pull-up. Also wakes from System OFF. |
| D7 | P1.12 | spare | | UART RX if a debug UART is wanted. Logs go over USB CDC ACM by default. |
| D8 | P1.13 | spare (SPI SCK) | | Reserved for a future flash chip or display. |
| D9 | P1.14 | spare (SPI MISO) | | |
| D10 | P1.15 | spare (SPI MOSI) | | |
| NFC1 pad | P0.09 | spare | | Usable as GPIO with `nfct-pins-as-gpios`. |
| NFC2 pad | P0.10 | spare | | |
| internal | P0.31 | VBAT_SENSE | in | AIN7 through the XIAO's divider. Enabled by P0.14. |
| internal | P0.14 | VBAT_SENSE_EN | out | Drive LOW to read. Never drive HIGH: it can push P0.31 past 3.6 V. |
| internal | P0.17 | CHG_STAT | in | LOW while charging. |
| internal | P0.13 | CHG_ISET | out | Hi-Z for 50 mA, LOW for 100 mA charge current. |
| BAT+ / BAT- pads | | BT1 | | LiPo. |
| SWDIO / SWCLK pads | | SWD | | Bring to test pads on the carrier for J-Link debugging. |

## Layout notes for the schematic and PCB

- Keep the BMP581 away from the XIAO's antenna end and its regulator; heat
  shifts the reading. Give it a foam cover against wind and light, and keep a
  vent hole in the enclosure above it.
- Mount the LSM6DSV16X near the board's centre, away from mounting screws.
- The XIAO antenna end should overhang the carrier or sit over a copper-free
  keepout.
- Both sensors need a stencil and reflow (or hot air); order a stencil with
  the boards.
