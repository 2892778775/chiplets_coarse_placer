# 3D IC Chiplets Coarse-Placement System v2

A constraint-driven coarse-placement engine for 3D IC heterogeneous integration (CoWoS, chiplet-based designs). Supports 3Dblox format input, D2D-aware expert placement, and SA-based optimization with hard/soft rule scoring.

## System Overview

This system reads 3Dblox-format designs (`.3dbx`, `.3dbv`, `.3dbo`, `.omap`) and produces optimized 2D/3D chiplet placements with:

- **Hard Rules** (must satisfy): no overlap, all-in-interposer, D2D PHY alignment & abutment, minimum spacing, LSI centering
- **Soft Rules** (optimization targets): vertical/horizontal symmetry, HBM/MEM side placement, IOD placement, D2D wire-length minimization

## Project Structure

```
ACF2.0/
├── chiplets_floorplan/
│   ├── core/                    # Core engine modules
│   │   ├── parser.py            # 3Dblox file parser (.3dbx, .3dbv, .3dbo, .omap, .connection)
│   │   ├── placer.py            # Expert + SA placement algorithms
│   │   ├── constraints.py     # Hard/soft constraint checker & scoring
│   │   ├── geometry.py          # Coordinate transforms, AABB, overlap resolution
│   │   ├── models.py            # Data models (ChipletDef, ChipletInst, AABB, D2DConnection, etc.)
│   │   ├── exporter.py          # Export PlacementSolution to 3Dblox format
│   │   ├── compaction.py        # Interposer sizing / compaction
│   │   ├── d2d_router.py        # D2D PHY alignment refinement
│   │   ├── dummy_filler.py      # Dummy die generation for gaps
│   │   └── simple_yaml.py       # Fallback YAML parser (if PyYAML unavailable)
│   ├── viz.py                   # Visualization: floorplan PNG, score table PNG, JSON/CSV export
│   ├── web/
│   │   └── app.py               # Flask web UI (REST API + frontend)
│   └── __init__.py
│
├── run_cli.py                   # Command-line automation entry
├── main.py                      # Web server entry (Flask)
├── CLI_README.md                # CLI usage guide
├── CoWoS_S/                     # Test case: 4 instances (SOC + 2x MEM)
├── CoWoS_L/                     # Test case: 8 instances (2x SOC + 2x HBM + 3x LSI)
├── README.md                    # This file
└── requirements.txt           # Python dependencies
```

## Quick Start

### CLI (Recommended for Automation)

```bash
# Expert placement (rule-based, fast, stable)
python run_cli.py --dbx CoWoS_S/CoWoS-S.3dbx --connection CoWoS_S/D2D.connection --output output_s/

# SA placement (Simulated Annealing, more iterations)
python run_cli.py --dbx CoWoS_L/CoWoS-L.3dbx --connection CoWoS_L/D2D.connection --output output_l/ --algorithm SA --sa-iterations 5000

# Skip optional steps for faster execution
python run_cli.py --dbx CoWoS_S/CoWoS-S.3dbx --connection CoWoS_S/D2D.connection --output output/ --skip-dummy --skip-d2d

# Full CLI options
python run_cli.py --help
```

See `CLI_README.md` for detailed CLI documentation.

### Web UI

```bash
pip install -r requirements.txt
python main.py
```

Then open `http://localhost:5000` in your browser.

### Python API

```python
from chiplets_floorplan.core.parser import Parser
from chiplets_floorplan.core.placer import Placer
from chiplets_floorplan.core.constraints import ConstraintChecker

# Parse design
parser = Parser()
design = parser.parse_design("CoWoS_S/CoWoS-S.3dbx", base_dir="CoWoS_S")
with open("CoWoS_S/D2D.connection", "r") as f:
    design.d2d_connections = parser.parse_connections(f.read())

# Run placement
placer = Placer(design, algorithm="Expert")
solution = placer.solve()

# Check constraints
checker = ConstraintChecker(design)
report = checker.check_all()
print(f"Valid: {report.is_valid}, Score: {report.total_score:.4f}")
```

## Placement Pipeline

```
Input (.3dbx + .connection)  →  Parse  →  Placement (Expert/SA)
                                    ↓
                          D2D Refinement (optional)
                                    ↓
                          Dummy Fill (optional)
                                    ↓
                          Compaction (Interposer sizing)
                                    ↓
                    Output: 3Dblox files + floorplan.png + score.json
```

## Hard Rules (H1–H7)

| ID | Rule | Description |
|----|------|-------------|
| H1 | No Overlap | Same-Z-layer chiplets must not overlap in XY |
| H2 | In-Interposer | All chiplets must be inside the Interposer boundary (enforced by compaction) |
| H3 | D2D Alignment | D2D PHY centers must be aligned on the same axis (X or Y) |
| H4 | D2D Abutment | Directly connected D2D PHYs must have zero-gap chiplet boundaries |
| H5 | Min Spacing | Adjacent chiplets must maintain `seal_ring + scribe_line` spacing (skip D2D pairs) |
| H6 | Centered MBR | All instance MBRs must align with the Interposer center |
| H7 | LSI Centering | Bridge LSI center must align with the midpoint of its D2D IP pair |

## Soft Rules (S1–S5)

| ID | Rule | Weight | Description |
|----|------|--------|-------------|
| S1 | Vertical Symmetry | 0.15 | Instances symmetric about horizontal center line |
| S2 | Horizontal Symmetry | 0.15 | Instances symmetric about vertical center line |
| S3 | HBM/MEM Placement | 0.20 | HBM/MEM placed on left or right side of SOC |
| S4 | IOD Placement | 0.20 | IOD placed on top or bottom side of SOC |
| S5 | D2D Length Minimize | 0.30 | Minimize total Manhattan wire length of D2D connections |

## Output Artifacts

After running `run_cli.py`, the output directory contains:

| File | Description |
|------|-------------|
| `*_export.3dbx` | Top-level 3Dblox design with updated Stack positions |
| `*_export.3dbv` | Master 3Dblov file referencing all chiplet definitions |
| `<chiplet>.3dbv` | Individual chiplet definition (size, seal_ring, etc.) |
| `<chiplet>.3dbo` | Chiplet object definitions (IP types and sizes) |
| `<chiplet>.omap` | Chiplet IP placement map (local coordinates) |
| `floorplan.png` | 2D visualization of all chiplets, D2D connections, and IPs |
| `score_table.png` | Tabular score report (hard + soft constraints) |
| `score.json` | Machine-readable JSON score report |
| `score.csv` | Spreadsheet-compatible CSV score report |

## Tested Design Cases

| Case | Chiplets | D2D Connections | LSI Bridges | Notes |
|------|----------|-------------------|-------------|-------|
| **CoWoS_S** | 3 (SOC, MEM, MEM) | 2 (direct) | 0 | No LSI, symmetric layout |
| **CoWoS_L** | 5 (SOC, HBM, LSI1, LSI2) | 3 (LSI-bridged) | 3 | LSI bridges connect SOC↔HBM pairs |

Both cases pass all hard constraints with `Expert` algorithm.

## Requirements

- Python 3.8 or higher
- See `requirements.txt` for package dependencies

## License

MIT License.
