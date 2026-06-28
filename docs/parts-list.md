# Flario Parts List

This document tracks the current locked-in and not-yet-locked components for Flario Rev A.

Status meanings:

- **Locked**: current design choice; use unless a later issue explicitly changes it.
- **Selected / verify**: preferred choice, but package, footprint, stock, or integration still needs confirmation.
- **Required / not locked**: required for the product, but no final part number has been selected yet.
- **Optional / TBD**: possible feature or layout provision, not required for Rev A.

## Locked / selected parts

| Subsystem | Function | Part number | LCSC / source | Status | Key notes |
|---|---|---|---|---|---|
| MCU / radio control | ESP32-S3 module | ESP32-S3-WROOM-1, exact memory variant TBD | TBD | Selected / verify | ESP32-S3-WROOM-1 family selected. Exact flash/PSRAM variant and exposed GPIO compatibility must be verified against final footprint and pin map. |
| Charger / power path | 1S LiPo charger / power-path IC | BQ25180YBGR | LCSC/JLCPCB part TBD | Locked | I2C charger. Charger INT omitted for Rev A; status polled over I2C. WCSP/DSBGA assembly risk must be accepted or alternative package chosen. |
| 3.3 V regulator | LDO regulator | AP2112K-3.3TRG1 | TBD | Locked | 3.3 V rail regulator. Verify output current and thermal margin against final load budget. |
| USB ESD | USB D+ / D- ESD protection | USBLC6-2SC6 | TBD | Locked | Protects USB data pair. Place close to USB-C connector. |
| Display | 2.7 in memory LCD | Sharp LS027B7DH01A | non-LCSC source likely | Locked | 2.7 in 400 x 240 Sharp Memory LCD. SPI interface. Requires FPC connector selection. |
| Barometer | Pressure sensor | MS5611 | TBD | Locked | Moved to I2C for Rev A. Polled; no INT line. Verify exact package/module and footprint. |
| IMU | 6-axis IMU | LSM6DSV16XTR | TBD | Locked | SPI interface. INT1 connected; INT2 omitted. |
| GNSS | GPS / GNSS receiver | u-blox NEO-M10 series | TBD | Locked | UART interface preferred. Exact module variant and antenna strategy still need footprint/source verification. |
| Audio amplifier | I2S class-D amplifier | MAX98357A or compatible | TBD | Selected / verify | I2S audio output. Confirm exact package/source and whether gain configuration pins/passives are needed. |
| Speaker | SMT speaker | KLJ-01304T-08R07W | LCSC C18186315 | Locked for Rev A | 8 ohm, 0.7 W, 87 dB SPL. Acceptable prototype choice; continue watching for 90-95 dB SPL alternatives. Prefer SMT speaker for production; add optional test pads for wired speaker evaluation if layout allows. |
| USB connector | USB-C receptacle | TYPE-C-31-M-12 or GT-USB-7079A | TBD | Selected / verify | USB 2.0 only. Final connector footprint/mechanical fit must be locked. |
| Battery connector | 1S LiPo connector | JST PH 3-pin horizontal SMT, e.g. S3B-PH-SM4-TB style | TBD | Selected / verify | Battery + thermistor/status pin approach still to be verified with charger requirements. |
| LoRa / FANET+ radio | LoRa transceiver | SX1262 module or IC path TBD | TBD | Selected / verify | FANET+ only. Exact final radio implementation, matching network/module choice, antenna connector, and regional SKU strategy need verification. |
| LoRa antenna | External antenna | 80 mm quarter-wave or SMA external antenna TBD | TBD | Selected / verify | External LoRa antenna preferred. Exact connector and enclosure approach not locked. |

## Required parts not locked yet

| Subsystem | Required part | Status | Notes / selection criteria |
|---|---|---|---|
| ESP32 support | Exact ESP32-S3-WROOM-1 memory variant | Required / not locked | Need exact flash/PSRAM choice. Must re-audit GPIO exposure and flash/PSRAM reserved pins before final schematic. |
| Display interconnect | FPC connector for LS027B7DH01A | Required / not locked | Need exact pitch, pin count, orientation, height, latch style, and footprint. |
| GPS antenna | GNSS antenna | Required / not locked | Decide internal patch/chip antenna vs external/u.FL. Must match NEO-M10 module requirements and enclosure. |
| LoRa antenna connector | SMA, u.FL, spring, or board connector | Required / not locked | External antenna planned, but exact RF connector/mechanical solution not locked. |
| Power switching / latch | Push-on / hold / force-off hardware circuit parts | Required / not locked | Discrete low-Iq latch still needs final schematic and MOSFET/resistor/capacitor part selection. Firmware HOLD pin is assigned, but hardware implementation is not final. |
| User input | Joystick / navigation switch | Required / not locked | Need exact 4-way or 5-way SMT joystick. Current GPIO map uses UP, DOWN, LEFT/power, RIGHT/force-off. |
| Power button | Main power / left button hardware | Required / not locked | Shared power sense / joystick LEFT concept selected. Need switch part, debounce/filter approach, and mechanical position. |
| Force-off button | Recessed force-off / right button hardware | Required / not locked | Shared force-off sense / joystick RIGHT concept selected. Actual force-off must work in hardware even if ESP32 is hung. |
| Battery | 1S LiPo pack | Required / not locked | Target around 3000 mAh for ~8 h runtime. Need size, connector, protection, NTC/thermistor compatibility, and supplier. |
| Battery protection | Protection circuit / protected cell choice | Required / not locked | Decide protected battery pack vs onboard protection. Must match charger and enclosure. |
| Passives | Resistors and capacitors | Required / not locked | Use 0603 preferred where practical. Need final values from datasheets and schematics. |
| Regulator passives | AP2112K capacitors | Required / not locked | Verify input/output capacitor values, dielectric, voltage rating, and placement. |
| Charger passives | BQ25180 capacitors / resistors | Required / not locked | Need full datasheet-derived values for IN, SYS, BAT, TS, ISET/ILIM, and any required pull-ups. |
| USB-C passives | CC resistors, VBUS cap, ESD layout details | Required / not locked | Need final USB-C receptacle pinout and USB 2.0 sink/device configuration. |
| Boost / 5 V rail | 5 V supply for audio or peripherals, if required | Required / not locked | Confirm whether design needs 5 V rail. Prior candidates included TPS63031/TLV61070 class parts, but no final rail architecture is locked here. |
| Test points | Programming, power, SPI, I2C, UART, reset/test pads | Required / not locked | Need DVT/debug access strategy and JLCPCB-friendly test pads. |
| Enclosure acoustics | Speaker acoustic port / membrane / gasket | Required / not locked | Speaker loudness depends strongly on enclosure cavity, venting, and weather resistance. |
| Mounting hardware | Screws, inserts, seals, adhesives | Required / not locked | Depends on final enclosure. |

## Explicit design decisions captured

- MS5611 moved to I2C.
- GPS remains on UART.
- LSM6DSV16XTR remains on SPI.
- LSM6DSV16XTR INT2 omitted.
- BQ25180 INT omitted; charger status will be polled over I2C.
- Power button sense shares a GPIO/function with joystick LEFT.
- Force-off sense shares a GPIO/function with joystick RIGHT.
- SMT speaker preferred over JST wired speaker for Rev A.
- Add optional test pads for a wired speaker if layout allows.

## Notes requiring follow-up

- Re-audit the full ESP32-S3-WROOM-1 GPIO map against the exact module datasheet and selected memory variant.
- Verify all LCSC/JLCPCB part numbers, stock, package, and assembly availability before BOM lock.
- Treat this file as the human-readable design parts list; the KiCad BOM will become the manufacturing source of truth once the schematic is complete.
