# airio

KiCad project for the airio carrier board: a Seeed XIAO nRF52840 soldered on
as a castellated module, with a BMP581 barometer, an LSM6DSV16X IMU, a piezo
beeper, a button, a 1S LiPo connector and SWD test pads. Signals follow the
pin map in [docs/airio-parts-and-pinmap.md](../../docs/airio-parts-and-pinmap.md).

Open `airio.kicad_pro` in KiCad 10. All files are saved in the KiCad 10
format. `airio.pdf` is a plot of the current schematic for reading without
KiCad.

| File | Contents |
|---|---|
| `airio.kicad_sch` | Schematic, one A4 sheet |
| `airio.kicad_pcb` | Two-layer board, 54 x 24 mm, 1.0 mm thick: parts placed, not routed |
| `airio.kicad_dru` | Custom DRC rules (see PCB below) |
| `airio-top.png` | Render of the top side |
| `airio.kicad_sym` | Project symbols: XIAO nRF52840, BMP581, LSM6DSV16X |
| `airio.pretty/` | Project footprints: `XIAO-nRF52840-Plus-SMD` (used), `XIAO-nRF52840-SMD` (previous), `Bosch_LGA-10_2x2mm_P0.5mm` |
| `sym-lib-table`, `fp-lib-table` | Add the two project libraries as `airio` |
| `airio.pdf` | Schematic plot |

## ERC

ERC in KiCad 10.0.7 reports 0 errors and 2 warnings. Both warnings are
sensor address straps tied to a supply rail, which is intended:

- U2 (BMP581) SDO to +3V3 selects I2C address 0x47.
- U3 (LSM6DSV16X) SDO/SA0 to GND selects I2C address 0x6A.

Exclude them in the ERC window (right-click, Exclude) if you want a clean run.

## Footprints

- **U1** is a XIAO nRF52840 Plus on `XIAO-nRF52840-Plus-SMD`, Seeed's footprint from
  [Seeed-Studio/OPL_Kicad_Library](https://github.com/Seeed-Studio/OPL_Kicad_Library)
  (commit b0035c5, CC-BY-SA-4.0), rotated so its main pads land where the standard
  XIAO's did. Pads 1-14 are the main castellations, 15-23 are D11-D19 on 1.27 mm
  half-pitch castellations, 24-27 the underside SWD pads (SWDIO, SWCLK, RST, GND)
  and 28-29 the battery pads (BAT+, BAT-). The Plus isn't on LCSC, so JLC can't
  place it: buy it from Seeed and hand solder it.
- **U2** uses `Bosch_LGA-10_2x2mm_P0.5mm`: 10 pads at 0.5 mm pitch, two on the
  pin 1 side and three on each adjacent side, numbered counter-clockwise from
  the top. Pad centres sit 0.775 mm from the body centre, pads are 0.30 x
  0.325 mm. It was cross-checked against four open-source BMP581 footprints,
  since Bosch's datasheet PDF couldn't be read here. Check it against
  section 8 of the datasheet before ordering. The BMP580 uses the same package.

Every symbol pin has a matching footprint pad (checked with KiCad's pcbnew
Python against the netlist).

## PCB

![Top side](airio-top.png)

- **Size:** 54 x 24 mm, two layers, 1.0 mm FR4. SMD parts sit on the top side
  for single-sided JLC reflow. J2-J4 are through-hole and need hand soldering or
  JLC's THT assembly. The six bare test pads are on the bottom.
- **Placement:** the XIAO sits on the right with its USB-C at the top edge. Its
  antenna end hangs over the bottom right, which has no copper on either layer
  (a rule area keeps tracks, vias and pours out). The piezo and JST sit along
  the top, the sensors in the lower middle, and the button in the bottom left.
  The BMP581 is kept away from the XIAO's regulator at the USB end. The ports
  sit in a 12 mm strip on the left edge: J3 (screen, 8-pin) down the edge, J2
  (GPS) above J4 (spare I2C) beside it. Pin 1 is GND on every port.
- **Routing:** not done. Tracks, vias and the ground pours were removed so
  the board can be routed by hand; the ratsnest shows the 64 connections still
  to make. The antenna keepout rule area and the custom rules stay.
- **Test pads:** TP1-TP6 are on the bottom in a 2.54 mm row (GND, 3V3, SWDIO,
  SWCLK, RST, VBAT, left to right seen from the top), under the battery, for a
  pogo-pin SWD adapter.
- **DRC:** KiCad 10.0.7 with schematic parity finds no footprint or parity
  errors. `airio.kicad_dru` allows the 0.15 mm pad gaps of ST's LGA-14 land
  pattern, sets a 0.3 mm copper-to-edge clearance, and allows 0.15 mm tracks,
  all within JLC's standard limits.

## Parts

Prices are LCSC, converted at 1.3993 CAD per USD (Bank of Canada, 2026-10-23),
excluding shipping and duty. Stock was read from LCSC's product API on
2026-10-24, and on 2026-10-25 for J2-J4 and U2. "Each" is the price at LCSC's smallest order quantity.

| Ref | Part | LCSC # | Qty | CAD each | Min order | LCSC stock |
|---|---|---|---|---|---|---|
| U1 | Seeed XIAO nRF52840 Plus (102010672) | Not on LCSC; Seeed | 1 | C$14.55 (US$10.40) | 1 | Not on LCSC (in stock at Seeed) |
| U2 | Bosch BMP581 | [C5362283](https://www.lcsc.com/product-detail/C5362283.html) | 1 | C$3.41 | 1 | 7,388 |
| U2 alt | Bosch BMP580 | [C22391138](https://www.lcsc.com/product-detail/C22391138.html) | 1 | C$1.70 | 1 | 14,944 |
| U3 | ST LSM6DSV16XTR | [C5267406](https://www.lcsc.com/product-detail/C5267406.html) | 1 | C$4.75 | 1 | 6,395 |
| BZ1 | Murata PKLCS1212E4001-R1 | [C113159](https://www.lcsc.com/product-detail/C113159.html) | 1 | C$1.12 | 1 | 7,183 |
| J1 | JST S2B-PH-SM4-TB(LF)(SN) | [C295747](https://www.lcsc.com/product-detail/C295747.html) | 1 | C$0.31 | 5 | 27,250 |
| J2, J4 | JST B4B-PH-K-S(LF)(SN), 4-pin vertical THT | [C131334](https://www.lcsc.com/product-detail/C131334.html) | 2 | C$0.064 | 10 | 111,650 |
| J3 | JST B8B-PH-K-S(LF)(SN), 8-pin vertical THT | [C157974](https://www.lcsc.com/product-detail/C157974.html) | 1 | C$0.122 | 5 | 61,155 |
| SW1 | XKB TS-1187A-B-A-B | [C318884](https://www.lcsc.com/product-detail/C318884.html) | 1 | C$0.03 | 20 | 923,940 |
| C1-C4 | Samsung CL10B104KO8NNNC, 100 nF 16 V X7R 0603 | [C66501](https://www.lcsc.com/product-detail/C66501.html) | 4 | C$0.013 | 50 | 796,150 |
| R1, R2 | UNI-ROYAL 0603WAF4701T5E, 4.7 kΩ 0603 | [C23162](https://www.lcsc.com/product-detail/C23162.html) | 2 | C$0.004 | 100 | 4,646,400 |

Ports J2-J4 follow `/mnt/project-files/airio/core-ports-spec.md`: pin 1 is GND on
all of them. GPS_RX is on D12 (P0.19) and SCR_DISP on D11 (P0.15), two of the XIAO Plus's
extra half-pitch pads, because the NFC pads sit inside the antenna keepout.

C1-C4 were changed from C1591 to C66501 because C1591 shows 0 stock at LCSC.
The BMP581 is back in stock at LCSC (2026-10-25); the BMP580 still fits the same footprint as a fallback.

## Still to do before ordering

- Check J1's polarity against the LiPo you'll use. Pin 1 is VBAT. JST PH
  battery packs are not consistent about which side is red.
- Check the BMP581 footprint against section 8 of the datasheet.

To point the KiCad MCP server at this project, set
`KICAD_MCP_PROJECT_DIR=hardware/airio` or ask Claude to call `kicad_set_project`.
