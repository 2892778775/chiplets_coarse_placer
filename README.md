# 3D IC Chiplets Coarse-Placement System v2

A constraint-driven coarse-placement engine for 3D IC heterogeneous integration
(CoWoS, CoW / chip-on-wafer, chiplet-based designs). It reads 3Dblox-format
designs (`.3dbx`, `.3dbv`, `.3dbo`, `.omap`) plus an optional D2D connection
file, and produces a legal, scored chiplet placement together with updated
3Dblox output and visual reports.

- **Hard rules (must satisfy):** no overlap, in-interposer, D2D PHY alignment
  & abutment, minimum spacing, centered MBR, LSI centering (H1–H7)
- **Soft rules (optimization targets):** vertical/horizontal symmetry,
  HBM/MEM side placement, IOD placement, D2D wire-length minimization (S1–S5)

## Installation

- Python 3.8 or higher.

```bash
pip install -r requirements.txt
```

Dependency notes:

- `PyYAML` is used to parse 3Dblox files. If it is not installed, the built-in
  fallback parser (`chiplets_floorplan/core/simple_yaml.py`) is used
  automatically, so the CLI still runs without it.
- `matplotlib` is only needed for image outputs (`floorplan.png`,
  `score_table.png`). Use `--no-images` to skip image generation.
- `Flask` / `flask-cors` are only needed for the optional Web UI.

## CLI Usage

```bash
# Expert placement (rule-based, fast, deterministic)
python run_cli.py --3dbx CoWoS_S/CoWoS-S.3dbx --connection CoWoS_S/D2D.connection --output output_s/

# Skip D2D refinement
python run_cli.py --3dbx design.3dbx --output output/ --skip-d2d

# Full option list
python run_cli.py --help
```

Main options (`--dbx` is an accepted alias of `--3dbx`):

| Option | Default | Description |
|--------|---------|-------------|
| `--3dbx` | (required) | Path to the top-level `.3dbx` input file |
| `--connection` | `""` | Path to the D2D `.connection` file |
| `--pi` | auto | Path to the `LSI.PI` affinity file (isolated chiplet → dominant). Defaults to `LSI.PI` next to the `.3dbx` when present |
| `--output` | `output` | Output directory for all artifacts |
| `--enclosure` | `500.0` | Minimum interposer enclosure margin (um) |
| `--skip-d2d` | off | Skip D2D PHY alignment refinement |
| `--no-images` / `--no-json` / `--no-csv` | off | Skip individual report artifacts |
| `--quiet` | off | Suppress non-essential console output |

Exit code is `0` when the final placement satisfies all hard rules, `1`
otherwise (and `2` on input errors).

See `CLI_README.md` for a Chinese version of the CLI guide.

## Placement Algorithm

### Expert (rule-based)

A deterministic, D2D-topology-driven constructive placer. It runs eight steps:

1. **Dominant identification** — instances are ranked by D2D degree; the
   highest-degree instance of each connected component becomes the dominant.
   LSI bridge chiplets get degree credit but are never placed as dominants.
2. **Design partition** — the base layer (Interposer, or the largest-area
   base def such as the RW wafer in CoW designs) is partitioned into one
   region per group.
3. **Isolation planning** — instances without D2D connections (IOD, IPD,
   DUMMY, …) are collected for later placement. Instances listed in the
   optional `LSI.PI` affinity file (e.g. eDTC / IVR) are bound to the group
   of the dominant instance they belong to.
4. **Dominant placement** — each dominant is placed in its region; the
   orientation is scored by D2D PHY alignment against its slaves, including a
   crossing penalty that avoids crossing D2D pairs.
5. **Slave placement** — every D2D-connected slave is abutted to its dominant
   (or to its LSI-bridged partner at the top-die level). All 8
   orientation/flip combinations are enumerated, filtered by PHY-axis
   compatibility, and the candidate minimizing total Manhattan PHY distance
   wins. The bridge LSI is centered exactly on the D2D PHY midpoint (H7).
6. **Isolated placement** — two phases, both after all same-group connected
   instances are placed. First, PI-bound isolated instances are placed
   *inside* their dominant's footprint, clear of LSI bridges and of each
   other. Then, free isolated instances are placed by a margin-aware
   corner search over the base footprint, preferring positions that **abut as
   many already-placed neighbors as possible** (pocket corners touching two
   or more sides win over single-side positions).
7. **Design merge** — groups with cross-group D2D connections are pulled
   together until the abutment/spacing rules are met.
8. **Centering** — the whole design is translated so the instance MBR center
   coincides with the base-layer center (H6).

A final overlap/spacing resolver only moves floating (isolated) instances;
anchored dominants, abutted slaves and LSI bridges are never displaced.
D2D-connected pairs are exempt from the spacing rule by construction.

See `docs/EXPERT_ALGORITHM.md` for the full step-by-step description,
including the key functions and constants of each step.

## Processing Pipeline

```
Input (.3dbx + .connection) → Parse → Placement (Expert)
                                   ↓
                     D2D refinement (optional, auto-reverted if it degrades)
                                   ↓
                     Compaction (interposer sizing / base-layer centering)
                                   ↓
                     Final constraint re-check
                                   ↓
        Output: 3Dblox files + floorplan.png + score_table.png + score.json/.csv
```

## Hard Rules (H1–H7)

| ID | Rule | Description |
|----|------|-------------|
| H1 | No Overlap | Same-Z-layer chiplets must not overlap in XY |
| H2 | In-Interposer | All chiplets must be inside the Interposer boundary (enforced by compaction) |
| H3 | D2D Alignment | D2D PHY centers must share the same X or Y axis (relaxed for LSI-bridged pairs) |
| H4 | D2D Abutment | Directly connected D2D chiplets must have zero-gap boundaries (relaxed for LSI-bridged pairs) |
| H5 | Min Spacing | Adjacent chiplets keep `seal_ring + scribe_line` spacing (D2D pairs exempt) |
| H6 | Centered MBR | The instance MBR must be centered on the base layer |
| H7 | LSI Centering | A bridge LSI's center must coincide with the midpoint of its D2D PHY pair (1e-6 tolerance) |

## Soft Rules (S1–S5)

| ID | Rule | Weight | Description |
|----|------|--------|-------------|
| S1 | Vertical Symmetry | 0.15 | Instances symmetric about the horizontal center line |
| S2 | Horizontal Symmetry | 0.15 | Instances symmetric about the vertical center line |
| S3 | HBM/MEM Placement | 0.20 | HBM/MEM placed on the left/right sides of the SOC |
| S4 | IOD Placement | 0.20 | IOD placed on the top/bottom sides of the SOC |
| S5 | D2D Length Minimize | 0.30 | Minimize total Manhattan length of D2D connections |

## Output Format

| File | Description |
|------|-------------|
| `<name>_export.3dbx` | Top-level 3Dblox design with updated `Stack` positions (sub-micron precision preserved) |
| `<name>_export.3dbv` | Master definition file including every chiplet `.3dbv` |
| `<chiplet>.3dbv` | Individual chiplet definition (size, shrink, thickness, seal ring, scribe line) |
| `<chiplet>.3dbo` | Chiplet object (IP) definitions |
| `<chiplet>.omap` | Chiplet IP placement map (local coordinates) |
| `floorplan.png` | 2D visualization: chiplets, D2D connections, IP positions |
| `score_table.png` | Tabular hard/soft constraint report |
| `score.json` | Machine-readable score report |
| `score.csv` | Spreadsheet-compatible score report (with rule weights appended) |

Exported `.3dbx` files re-parse cleanly: running the constraint checker on
the exported files reproduces the reported score.

## Verified Test Cases

| Case | Instances | D2D Connections | LSI Bridges | Expert Result |
|------|-----------|-----------------|-------------|---------------|
| **CoWoS_S** (bundled) | 3 + Interposer | 2 (direct) | 0 | Valid, score 0.9333 |
| **CoWoS_L** (bundled) | 7 + Interposer | 3 (LSI-bridged) | 3 | Valid, score 0.9666 |
| **All-In-One (CoW)** | 44 + RW wafer base | 18 (LSI-bridged) | 18 | Valid, score 0.9358 |

All cases pass every hard rule with the Expert placer, which is fully
deterministic: repeated runs produce identical layouts and scores. The CoW
case also reads an `LSI.PI` affinity file (one `child,parent` pair per line)
that binds its eDTC / IVR isolated chiplets to their dominant SoIC die:
PI-bound chiplets are placed inside the dominant's footprint (clear of LSI
bridges and of each other), while the remaining free isolated chiplets
(IOD2) abut their already-placed neighbors on as many sides as possible
(e.g. one side against IOD1 and another against HBM).

## Project Structure

```
├── chiplets_floorplan/
│   ├── core/
│   │   ├── parser.py          # 3Dblox parser (.3dbx/.3dbv/.3dbo/.omap/.connection)
│   │   ├── placer.py          # ExpertPlacer (8-step deterministic construction)
│   │   ├── constraints.py     # Hard/soft constraint checker & scoring
│   │   ├── geometry.py        # Coordinate transforms, AABB ops, overlap resolution
│   │   ├── models.py          # Data models (ChipletDef, ChipletInst, AABB, D2DConnection, …)
│   │   ├── exporter.py        # PlacementSolution → 3Dblox writer
│   │   ├── compaction.py      # Interposer sizing / base-layer centering
│   │   ├── d2d_router.py      # D2D PHY alignment refinement
│   │   └── simple_yaml.py     # Fallback YAML parser (when PyYAML is absent)
│   ├── viz.py                 # floorplan.png / score_table.png / score.json / score.csv
│   └── web/app.py             # Optional Flask Web UI (python -m chiplets_floorplan.web.app)
│                              # Uploads: .3dbx, D2D.connection, LSI.PI affinity file
├── docs/
│   └── EXPERT_ALGORITHM.md    # Step-by-step Expert placer description
├── run_cli.py                 # CLI entry point
├── CLI_README.md              # CLI guide (Chinese)
├── CoWoS_S/                   # Bundled test case
├── CoWoS_L/                   # Bundled test case
└── requirements.txt
```

## Python API

```python
from chiplets_floorplan.core.parser import Parser
from chiplets_floorplan.core.placer import ExpertPlacer
from chiplets_floorplan.core.constraints import ConstraintChecker

parser = Parser()
design = parser.parse_design("CoWoS_S/CoWoS-S.3dbx")
with open("CoWoS_S/D2D.connection", "r", encoding="utf-8") as f:
    design.d2d_connections = parser.parse_connections(f.read())

solution = ExpertPlacer(design).solve()
report = ConstraintChecker(design).check_all()
print(f"Valid: {report.is_valid}, Score: {report.total_score:.4f}")
```

## License

MIT License.
