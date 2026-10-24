# KiCad MCP server

The repo ships a project-scoped MCP config (`/.mcp.json`) that gives Claude Code
access to the Flario KiCad project through
[KiCad MCP Pro](https://github.com/oaslananka/kicad-mcp-pro) (MIT, v4.3.0).
It can read schematics and PCBs, run ERC/DRC through `kicad-cli`, check BOM and
DFM, and, when you allow it, make controlled edits and produce manufacturing
outputs.

## Why this server

| Server | Notes |
|---|---|
| **oaslananka/kicad-mcp-pro** (chosen) | Actively released, one-command `uvx` install, read-only by default, works on Windows and Linux. Targets KiCad 10. |
| mixelpixx/KiCAD-MCP-Server | Most stars, KiCad 9+, but needs a Node build plus KiCad's Python bindings; new work is moving to its Rust successor, Konnect. Use it if you stay on KiCad 9. |
| mixelpixx/Konnect | KiCad 10 plugin that drives the running GUI over the IPC API. AGPL, still beta. |
| Seeed-Studio/kicad-mcp-server | Lightweight review-only option (KiCad 8+). |

## Setup (each dev machine)

1. Install **KiCad 10** and make sure `kicad-cli` is on `PATH`
   (Windows: `C:\Program Files\KiCad\10.0\bin`). Otherwise set `KICAD_CLI_PATH`.
2. Install **uv**, which provides `uvx`:
   - Windows: `powershell -c "irm https://astral.sh/uv/install.ps1 | iex"`
   - Linux: `curl -LsSf https://astral.sh/uv/install.sh | sh`
3. Check the server can see KiCad: `uvx kicad-mcp-pro@4.3.0 doctor`
4. Open Claude Code in the repo root and approve the `kicad` server when
   prompted. `claude mcp list` should show it connected.

## Configuration

Both settings can be overridden with environment variables before launching
Claude Code.

| Variable | Default | Meaning |
|---|---|---|
| `KICAD_MCP_PROJECT_DIR` | `hardware` | Folder holding the `.kicad_pro`. If the relative path isn't picked up, set an absolute path, or ask Claude to call `kicad_set_project`. |
| `KICAD_MCP_OPERATING_MODE` | `readonly` | `readonly`, `write` (controlled edits), `manufacturing` (Gerbers and release outputs), `experimental`. |
| `KICAD_CLI_PATH` | from `PATH` | Full path to `kicad-cli`. |

Example, allowing edits for one session on Linux:

```bash
KICAD_MCP_OPERATING_MODE=write claude
```

The version is pinned in `.mcp.json`; bump it deliberately after reading the
release notes.
