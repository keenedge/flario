# airio v0: parts list and XIAO nRF52840 pin map

airio is a small BLE-only vario, separate from the ESP32-S3 Flario Rev A in
[parts-list.md](parts-list.md). v0 is a carrier PCB with a Seeed XIAO nRF52840
soldered on as a castellated module, plus a baro, an IMU and a piezo beeper.
Firmware is Zephyr (nRF Connect SDK) in [`airio/firmware`](../airio/firmware).

Decided 2026-10-24: XIAO nRF52840, BMP581, LSM6DSV16X. Ports changed to JST SH on 2026-10-26; J1 battery stays JST PH.

## Parts

Prices are LCSC, converted at 1.40 CAD per USD (2026-10-23 close), excluding
shipping and duty. "Live" rows were checked against LCSC's product API on
2026-10-24; "page" rows come from LCSC product page snapshots and should be
rechecked before ordering. "From" is LCSC's lowest tier, so qty 1 costs more.

| Ref | Function | Part | LCSC # | CAD each | LCSC stock | Source | Notes |
|---|---|---|---|---|---|---|---|
| U1 | MCU + BLE + USB + charger | Seeed XIAO nRF52840 (102010448) | [C17209540](https://www.lcsc.com/product-detail/C17209540.html) | from C$14.82 | 158 | page | Castellated module. Onboard BQ25101 charger, 3.3 V LDO, USB-C, UF2 bootloader. Non-Sense version (no onboard IMU). |
| U2 | Barometer | Bosch BMP581 | [C5362283](https://www.lcsc.com/product-detail/C5362283.html) | C$3.41 | 7,388 | live 2026-10-25 | LGA-10 2x2 mm. I2C. Back in stock. |
| U2-alt | Barometer (likely drop-in) | Bosch BMP580 | [C22391138](https://www.lcsc.com/product-detail/C22391138.html) | C$1.70 | 14,944 | live | Same 2x2 mm LGA-10 package and specs per Bosch. Pinout not yet checked against the BMP580 datasheet. Zephyr's `bmp581` driver accepts chip IDs 0x50 and 0x51, so firmware should work unchanged. |
| U3 | 6-axis IMU | ST LSM6DSV16XTR | [C5267406](https://www.lcsc.com/product-detail/C5267406.html) | C$4.75 | 6,395 | live | LGA-14 2.5x3 mm. I2C. On-chip sensor fusion (SFLP). |
| J2 | GPS port | JST SM04B-SRSS-TB(LF)(SN), 4-pin SH side-entry SMD | [C160404](https://www.lcsc.com/product-detail/C160404.html) | from C$0.303 | 63,590 | live 2026-10-26 | 1 GND, 2 +3V3, 3 GPS RX (from D10), 4 GPS TX (to D9). For the SparkFun MAX-M10S breakout. |
| J3 | Screen port | JST SM08B-SRSS-TB(LF)(SN), 8-pin SH side-entry SMD | [C160407](https://www.lcsc.com/product-detail/C160407.html) | from C$0.457 | 194,320 | live 2026-10-26 | 1 GND, 2 +3V3, 3 VBAT, 4 DISP (U4 P2), 5 CS (D2), 6 CLK (D1), 7 MOSI (D0), 8 SCR_DC (D8). For the Adafruit 2.7" Sharp memory LCD breakout (pin 8 can drive EXTCOMIN) or a cabled ST7305 adapter (pin 4 = RES, pin 8 = D/C). |
| J6 | ST7305 panel FPC | HCTL HC-FPC-05-09-24RLTAG, 24-pin 0.5 mm, dual contact, right angle | [C5213757](https://www.lcsc.com/product-detail/C5213757.html) | C$0.19 (5+) | 50,645 | live 2026-10-26 | For a bare 2.9" 168x384 ST7305 reflective panel (Osptek YDP290H001-V3, Good Display GDTL029T51). 5/15/16 +3V3, 17 GND, 10 RES (SCR_DISP), 11 D/C (D8), 12 CS (D2), 13 SCLK (D1), 14 SDI (D0), 18/20/21/22/24 VGH/VIV/AVDD/VNV/VGL to C6-C10; 1-4, 6-9, 19, 23 not connected. Pinout from search extracts of Osptek's spec, not the PDF itself. Footprint is a Hirose FH12 placeholder: check pads against the HCTL drawing. |
| J4 | Spare I2C port (Qwiic / STEMMA QT pinout) | JST SM04B-SRSS-TB(LF)(SN), 4-pin SH side-entry SMD | [C160404](https://www.lcsc.com/product-detail/C160404.html) | from C$0.303 | 63,590 | live 2026-10-26 | 1 GND, 2 +3V3, 3 SDA, 4 SCL. Shares R1/R2 pull-ups; no pull-ups at the port. |
| J5 | Buzzer port | JST SM02B-SRSS-TB(LF)(SN), 2-pin SH side-entry SMD | [C160402](https://www.lcsc.com/product-detail/C160402.html) | from C$0.187 | 73,300 | live 2026-10-26 | 1 BUZZ_A, 2 BUZZ_B. Off-board piezo, driven differentially. Replaces the onboard PKLCS1212E4001 (BZ1) on 2026-10-26. |
| C1-C4 | Sensor decoupling, 100 nF 0603 X7R | Samsung CL10B104KB8NNNC | [C1591](https://www.lcsc.com/product-detail/C1591.html) | from C$0.01 | conflicting on page | page | One each on BMP581 VDD, BMP581 VDDIO, LSM6DSV16X VDD, LSM6DSV16X VDDIO. JLC basic part. |
| C6-C12 | ST7305 panel capacitors, 1 µF 0603 X5R 50 V | Samsung CL10A105KB8NNNC | [C15849](https://www.lcsc.com/product-detail/C15849.html) | from C$0.02 | 722,400 | live 2026-10-26 | C6 VGH, C7 VGL, C8 AVDD, C9 VIV, C10 VNV (panel-generated rails), C11 VCI, C12 IOVCC. Typical Sitronix values; confirm against the panel spec. 50 V parts keep DC-bias derating on VGH/VGL acceptable. |
| R1, R2 | I2C pull-ups, 4.7 kΩ 0603 | UNI-ROYAL 0603WAF4701T5E | [C23162](https://www.lcsc.com/product-detail/C23162.html) | from C$0.002 | 4,646,600 | page | SDA and SCL to 3V3. |
| U4 | I2C I/O expander | TI TCA9534PWR, TSSOP-16 | [C783615](https://www.lcsc.com/product-detail/C783615.html) | C$1.31 | 11,270 | live 2026-10-25 | Address 0x20 (A0-A2 to GND). P0 BTN, P1 BARO_INT, P2 SCR_DISP, P3-P7 spare. Open-drain INT to D6. No internal pull-ups; set unused pins as outputs in firmware. |
| R3 | Expander INT pull-up, 10 kΩ 0603 | UNI-ROYAL 0603WAF1002T5E | [C25804](https://www.lcsc.com/product-detail/C25804.html) | from C$0.003 | 567,800 | live 2026-10-25 | EXP_INT to 3V3. |
| R4 | Button pull-up, 10 kΩ 0603 | UNI-ROYAL 0603WAF1002T5E | [C25804](https://www.lcsc.com/product-detail/C25804.html) | from C$0.003 | 567,800 | live 2026-10-25 | BTN to 3V3; the TCA9534 has no internal pull-ups. |
| C5 | Expander decoupling, 100 nF 0603 | Samsung CL10B104KO8NNNC | [C66501](https://www.lcsc.com/product-detail/C66501.html) | from C$0.013 | 796,150 | live 2026-10-24 | On U4 VDD. |
| SW1 | User / power button | XKB TS-1187A-B-A-B | [C318884](https://www.lcsc.com/product-detail/C318884.html) | from C$0.03 | 923,940 | live 2026-10-24 | Tactile switch to GND on U4 P0, pulled up by R4. |
| BT1 | 1S LiPo, about 150 mAh | Not on LCSC | | | | | Solder to the XIAO's BAT+/BAT- pads. Size from the 8 h power estimate with 2x margin. |

## Pin map

All signals use the XIAO's 3.3 V rail. The 1.8 V rail idea from the power
discussion doesn't apply to v0 because the XIAO has a fixed 3.3 V regulator;
revisit it for a bare-chip v1.

| XIAO pin | nRF52840 | airio signal | Direction | Notes |
|---|---|---|---|---|
| D0 | P0.02 | SCR_MOSI | out | J3 pin 7, J6 pin 14. SPI data to the display. |
| D1 | P0.03 | SCR_CLK | out | J3 pin 6, J6 pin 13. SPI clock to the display. |
| D2 | P0.28 | SCR_CS | out | J3 pin 5, J6 pin 12. Chip select: active high on Sharp memory LCDs, active low on the ST7305. |
| D3 | P0.29 | I2C_SDA | io | Zephyr `i2c1`. BMP581 at 0x47, LSM6DSV16X at 0x6A, TCA9534 at 0x20; J4 is a Qwiic port on the same bus. |
| D4 | P0.04 | I2C_SCL | out | Zephyr `i2c1`, 400 kHz. |
| D5 | P0.05 | EXP_INT | in | U4 TCA9534 INT, open drain, active low, R3 pull-up. Fires on any expander input change and wakes from System OFF. |
| D6 | P1.11 | BUZZ_A | out | J5 pin 1. PWM1 channel 0. (PWM0 is used by the XIAO's LED.) |
| D7 | P1.12 | BUZZ_B | out | J5 pin 2. PWM1 channel 1, inverted, so the piezo sees twice the swing. |
| D8 | P1.13 | SCR_DC | out | J6 pin 11 (ST7305 D/C) and J3 pin 8. Toggles on every SPI command, so it is a direct MCU pin. |
| D9 | P1.14 | GPS_TX | in | J2 pin 4. GPS TX into the MCU's UART RX. Logs go over USB CDC ACM. |
| D10 | P1.15 | GPS_RX | out | J2 pin 3. MCU UART TX to the GPS RX. |
| NFC1 pad | P0.09 | unused | | Inside the antenna keepout. |
| NFC2 pad | P0.10 | unused | | Inside the antenna keepout. |
| U4 P0 | | BARO_INT | in | BMP581 INT, data-ready, push-pull active high. Read over I2C after EXP_INT. |
| U4 P1 | | IMU_INT1 | in | LSM6DSV16X INT1. Use the FIFO watermark interrupt in latched mode so it fires a few times a second. |
| U4 P2 | | SCR_DISP | out | J3 pin 4, J6 pin 10. Sharp DISP enable, or ST7305 RES (active low). |
| U4 P3 | | BTN | in | SW1 to GND, R4 pull-up. Read over I2C after EXP_INT. |
| U4 P4-P7 | | spare | | Configure as outputs so the inputs don't float. |
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
