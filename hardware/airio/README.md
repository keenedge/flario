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
| `airio.kicad_pcb` | Two-layer board, 1.0 mm thick: layout and routing in progress |
| `airio.kicad_dru` | Custom DRC rules (see PCB below) |
| `airio-top.png` | Render of an earlier placement |
| `airio.kicad_sym` | Project symbols: XIAO nRF52840, BMP581, LSM6DSV16X |
| `airio.pretty/` | Project footprints: `XIAO-nRF52840-SMD`, `Bosch_LGA-10_2x2mm_P0.5mm` |
| `sym-lib-table`, `fp-lib-table` | Add the two project libraries as `airio` |
| `airio.pdf` | Schematic plot |

## ERC

ERC in KiCad 10.0.7 reports 0 errors and 2 warnings. Both warnings are
sensor address straps tied to a supply rail, which is intended:

- U2 (BMP581) SDO to +3V3 selects I2C address 0x47.
- U3 (LSM6DSV16X) SDO/SA0 to GND selects I2C address 0x6A.

Exclude them in the ERC window (right-click, Exclude) if you want a clean run.

## Footprints

- **U1** uses Seeed's `XIAO-nRF52840-SMD` footprint from
  [Seeed-Studio/OPL_Kicad_Library](https://github.com/Seeed-Studio/OPL_Kicad_Library)
  (commit 3be8215, CC-BY-SA-4.0). Pads 1-14 are the castellated edge, 15-18 the
  underside SWD pads (SWDIO, SWCLK, RST, GND), 19-20 the battery pads (BAT+,
  BAT-) and 21-22 NFC1/NFC2. The project symbol's underside pins were
  renumbered to Seeed's order so they land on the right pads.
- **U2** uses `Bosch_LGA-10_2x2mm_P0.5mm`: 10 pads at 0.5 mm pitch, two on the
  pin 1 side and three on each adjacent side, numbered counter-clockwise from
  the top. Pad centres sit 0.775 mm from the body centre, pads are 0.30 x
  0.325 mm. It was cross-checked against four open-source BMP581 footprints,
  since Bosch's datasheet PDF couldn't be read here. Check it against
  section 8 of the datasheet before ordering. The BMP580 uses the same package.

Every symbol pin has a matching footprint pad (checked with KiCad's pcbnew
Python against the netlist).

## Display options

J3 is a cable port for a display breakout (Sharp memory LCD, or an ST7305
adapter). J6 is an on-board 24-pin 0.5 mm FPC for a bare 2.9" 168x384 ST7305
reflective panel (Osptek YDP290H001-V3, Good Display GDTL029T51), with its
seven 1 µF capacitors C6-C12. Both share SCR_CLK, SCR_MOSI, SCR_CS, SCR_DISP
(ST7305 RES) and SCR_DC (D8). Fit one display at a time.

J6's footprint is a placeholder (`Hirose_FH12-24S-0.5SH`): check the pads
against the HCTL HC-FPC-05-09-24RLTAG drawing before layout. The J6 pinout and
capacitor values come from search extracts of the panel spec, not the PDF
itself; confirm them too. `airio.pdf` has not been re-plotted for J6.

## PCB

![Top side](airio-top.png)

- **Size:** 54 x 24 mm, two layers, 1.0 mm FR4. SMD parts sit on the top side
  for single-sided JLC reflow. J2-J5 are SMD JST SH, so the
  whole board can be reflowed. The six bare test pads are on the bottom.
- **Placement and routing:** being re-laid out and routed by hand in KiCad (work
  in progress). `airio-top.png` shows the earlier scripted placement, not the
  current layout. The antenna keepout rule area and the custom rules stay.
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
2026-10-24, on 2026-10-25 for U2, U4 and R3/R4, and on 2026-10-26 for J2-J5. "Each" is the price at LCSC's smallest order quantity.

| Ref | Part | LCSC # | Qty | CAD each | Min order | LCSC stock |
|---|---|---|---|---|---|---|
| U1 | Seeed XIAO nRF52840 (102010448) | [C17209540](https://www.lcsc.com/product-detail/C17209540.html) | 1 | C$15.18 | 1 | 158 |
| U2 | Bosch BMP581 | [C5362283](https://www.lcsc.com/product-detail/C5362283.html) | 1 | C$3.41 | 1 | 7,388 |
| U2 alt | Bosch BMP580 | [C22391138](https://www.lcsc.com/product-detail/C22391138.html) | 1 | C$1.70 | 1 | 14,944 |
| U3 | ST LSM6DSV16XTR | [C5267406](https://www.lcsc.com/product-detail/C5267406.html) | 1 | C$4.75 | 1 | 6,395 |
| J1 | JST S2B-PH-K-S(LF)(SN), 2-pin PH side-entry THT, on the back | [C173752](https://www.lcsc.com/product-detail/C173752.html) | 1 | C$0.056 | 20 | 202,040 |
| J2, J4 | JST SM04B-SRSS-TB(LF)(SN), 4-pin SH side-entry SMD | [C160404](https://www.lcsc.com/product-detail/C160404.html) | 2 | C$0.303 | 5 | 63,590 |
| J3 | JST SM08B-SRSS-TB(LF)(SN), 8-pin SH side-entry SMD | [C160407](https://www.lcsc.com/product-detail/C160407.html) | 1 | C$0.457 | 5 | 194,320 |
| J5 | JST SM02B-SRSS-TB(LF)(SN), 2-pin SH side-entry SMD (buzzer) | [C160402](https://www.lcsc.com/product-detail/C160402.html) | 1 | C$0.187 | 5 | 73,300 |
| SW1 | XKB TS-1187A-B-A-B | [C318884](https://www.lcsc.com/product-detail/C318884.html) | 1 | C$0.03 | 20 | 923,940 |
| C1-C4 | Samsung CL10B104KO8NNNC, 100 nF 16 V X7R 0603 | [C66501](https://www.lcsc.com/product-detail/C66501.html) | 4 | C$0.013 | 50 | 796,150 |
| R1, R2 | UNI-ROYAL 0603WAF4701T5E, 4.7 kΩ 0603 | [C23162](https://www.lcsc.com/product-detail/C23162.html) | 2 | C$0.004 | 100 | 4,646,400 |
| U4 | TI TCA9534PWR I/O expander, TSSOP-16 | [C783615](https://www.lcsc.com/product-detail/C783615.html) | 1 | C$1.31 | 1 | 11,270 |
| R3, R4 | UNI-ROYAL 0603WAF1002T5E, 10 kΩ 0603 | [C25804](https://www.lcsc.com/product-detail/C25804.html) | 2 | C$0.003 | 100 | 567,800 |
| C5 | Samsung CL10B104KO8NNNC, 100 nF 16 V X7R 0603 | [C66501](https://www.lcsc.com/product-detail/C66501.html) | 1 | C$0.013 | 50 | 796,150 |

Ports J2-J4 follow `/mnt/project-files/airio/core-ports-spec.md`: pin 1 is GND on
all of them. J2-J5 are JST SH (1.0 mm) side-entry SMD, so JLC can reflow them; they
need to sit at the board edge, and SH leads are best bought pre-crimped. J4 matches the
Qwiic / STEMMA QT pinout. The buzzer is off-board on J5. The NFC pads sit inside the
antenna keepout, so they are unused. U4, a TCA9534 I2C expander at 0x20, carries BTN
(P3), BARO_INT (P0), SCR_DISP (P2) and IMU_INT1 (P1); its INT output on D5 wakes the
XIAO from System OFF. See the pin map for the XIAO pin assignment.

C1-C4 were changed from C1591 to C66501 because C1591 shows 0 stock at LCSC.
The BMP581 is back in stock at LCSC (2026-10-25); the BMP580 still fits the same footprint as a fallback.

## J1 on the back

J1 is a through-hole JST PH (S2B-PH-K-S) mounted on the bottom side, so the
battery lead runs along the back. Hand solder it from the top after JLC's
single-sided assembly, and mark it "do not place" in the JLC BOM. Its pins
poke about 1.5 mm through the top side, so keep those spots clear.

## Still to do before ordering

- Check J1's polarity against the LiPo you'll use. Pin 1 is VBAT. JST PH
  battery packs are not consistent about which side is red.
- Check the BMP581 footprint against section 8 of the datasheet.

To point the KiCad MCP server at this project, set
`KICAD_MCP_PROJECT_DIR=hardware/airio` or ask Claude to call `kicad_set_project`.
