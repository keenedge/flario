# airio

KiCad project for the airio carrier board: a Seeed XIAO nRF52840 soldered on
as a castellated module, with a BMP581 barometer, an LSM6DSV16X IMU, a piezo
beeper, a button, a 1S LiPo connector and SWD test pads. Signals follow the
pin map in [docs/airio-parts-and-pinmap.md](../../docs/airio-parts-and-pinmap.md).

Open `airio.kicad_pro` in KiCad 10. The schematic is saved in the KiCad 7 file
format, and KiCad upgrades it on the first save. `airio.pdf` is a plot of the
current schematic for reading without KiCad.

| File | Contents |
|---|---|
| `airio.kicad_sch` | Schematic, one A4 sheet |
| `airio.kicad_sym` | Project symbols: XIAO nRF52840, BMP581, LSM6DSV16X |
| `sym-lib-table` | Adds `airio.kicad_sym` to the project as library `airio` |
| `airio.pdf` | Schematic plot |

## Still to do before layout

- **U1 footprint.** No footprint is assigned yet. It needs the 14 castellated
  pads plus the underside pads (BAT+, BAT-, SWDIO, SWCLK, RST, GND, NFC1, NFC2)
  that the carrier connects to. Use Seeed's KiCad footprint or draw one from
  the XIAO drawing.
- **U2 footprint.** No footprint is assigned yet. Draw the LGA-10 2x2 mm land
  pattern from section 8.2 of the BMP581 datasheet. The BMP580 uses the same one.
- Run ERC in KiCad 10. Connectivity was checked from a netlist export, but ERC
  has not been run yet.

To point the KiCad MCP server at this project, set
`KICAD_MCP_PROJECT_DIR=hardware/airio` or ask Claude to call `kicad_set_project`.
