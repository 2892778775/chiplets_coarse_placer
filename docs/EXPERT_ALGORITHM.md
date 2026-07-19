# Expert Placer Algorithm

The Expert placer (`chiplets_floorplan/core/placer.py`, class `ExpertPlacer`) is
a deterministic, D2D-topology-driven constructive placer. `solve()` runs an
8-step pipeline, then a final constraint check and interposer sizing. There is
no randomness anywhere: every set iteration is sorted, so identical input
always produces an identical layout and score.

Key constants (top of `placer.py`):

| Constant | Value | Meaning |
|----------|-------|---------|
| `SPACING_EPSILON` | 1.0 | Extra clearance added to every margin pair (um) |
| `ABUT_TOLERANCE` | 1e-3 | Gap window within which two instances count as abutting (um) |
| `DEFAULT_ENCLOSURE` | 500.0 | Default interposer enclosure margin (um) |
| `EDGE_NEAR_THRESHOLD` / `EDGE_FAR_THRESHOLD` | 0.35 / 0.65 | Fraction of chiplet width/height classifying an IP to an edge |
| `CENTER_ALIGNMENT_TOL` | 0.1 | Tolerance for the "balanced PHY" check |

Margin between two instances is always
`mm = max(seal_ring_i) + max(scribe_line_i) + max(seal_ring_j) + max(scribe_line_j) + SPACING_EPSILON`.
Two instances **abut** when their gap equals `mm` within `ABUT_TOLERANCE`.

## Step 1 — Dominant instances recognition

`_step1_dominant_instances()`

- Score every non-base instance's D2D degree (+1 as connection source/target,
  +2 as an LSI bridge).
- **LSI bridges are identified first** (the `lsi_inst` slot of a connection,
  or a reference name containing "LSI") so they are never mistaken for slaves
  or isolated chiplets.
- **Dominants**: reference names containing `SOC`/`SOIC`; if the design has no
  SoC, the max-degree instances; fallback is every instance with degree > 0.
- **Slaves**: connected non-dominants, assigned to the dominant they have the
  most connections to (ties broken by degree); slaves with no link to any
  dominant join the nearest dominant by initial position.
- **Isolated**: degree 0 and neither dominant nor slave (e.g. IOD2, eDTC, IVR).

## Step 2 — Design partition

`_step2_design_partition()`

- Reference size: the `Interposer` def, or the largest-area def (e.g. the `RW`
  wafer in CoW designs).
- The base layer is partitioned into one group per dominant: split along the
  long axis for N=2, a near-square grid for N>2.
- Dominants enter groups in sorted-name order; slaves follow their dominant;
  each LSI bridge joins its connection source's (otherwise target's) group.

## Step 3 — Isolate instances plan

`_step3_isolate_instances_plan()`

- Isolated instances listed in the **`LSI.PI` affinity file** (one
  `child,parent` pair per line, e.g. eDTC / IVR) join the group of their
  dominant instance directly.
- The remaining free isolated instances are bucketed by reference type and
  distributed evenly across groups (deterministic sorted order).

## Step 4 — Dominant instance placement

`_step4_dominant_placement()`

- `_analyze_dominant_ips()` builds the PHY edge histogram of each dominant,
  checks balance (symmetric IP distribution within `CENTER_ALIGNMENT_TOL`) and
  partner-type uniformity.
- Placement edge selection: same-type + balanced → center; cross-group
  connections → face the partner group (the axis with the larger center
  separation wins); otherwise the PHY-densest edge.
- Orientation: `_score_dominant_orientations()` brute-forces
  4 orientations × 3 flips, scoring desired-edge matches (+2 match, −2
  opposite, +0.5 adjacent). When two or more cross-group connections link the
  same die pair, an **X-crossing penalty** keeps the PHY order along the
  abutment axis (no crossed D2D pairs).

## Step 5 — Slave instances placement

`_step5_slave_placement()` → `_place_slave_for_connection()`

- For every (orientation, flip) candidate the slave abuts its dominant on the
  dominant's PHY-edge side, with PHY centers aligned exactly along the
  abutment axis; the candidate with the smallest total Manhattan PHY-to-PHY
  distance wins. This reproduces the mirror conventions of manual placements
  (MY / R180) without per-type hard-coded rules.
- Helpers: `_phy_axis_compatible()` (axis filter) and
  `_inset_for_corner_clearance()` (corner spacing inset).
- Slaves without connections, or failed candidates, fall back to
  `_place_slave_near_dominant()`.

## LSI bridge placement

`_place_lsi_internal()` — runs once before Step 6 and again after Step 7.

- Every LSI is centered exactly on the global midpoint of its two bridged PHYs
  (hard rule H7), orientation `R0` / no flip. The LSI sits on a lower Z layer
  (z=250); the two top dies (z=500) must still abut each other directly.

## Step 6 — Isolate instances placement

`_step6_isolate_placement()`

- Search area: the base footprint pre-offset by (anchored-MBR center − base
  center), so candidates stay inside the base after Step 8's translation. The
  anchored MBR contains only already-placed instances (dominants, slaves,
  LSI); unplaced isolated instances must not pollute it.
- Empty groups (no dominant/slave) get an even grid distribution
  (`_place_isolated_in_empty_group()`).
- **Phase A — PI-bound isolated** (`_place_pi_isolated()`): placed *inside*
  their dominant's footprint, clearing LSI bridges and each other
  (margin-aware first-fit over candidate corners). Failure falls back to free
  abut placement.
- **Phase B — free isolated** (`_place_isolated_margin_aware()`): candidate
  positions come from the area boundary and every placed neighbor's edges
  ± margins. Feasibility = `inflate(mm)` overlaps nothing on overlapping Z
  layers. Ranking prefers the position that **abuts the most placed
  neighbors** (per-neighbor side tests with axis overlap: left/right/top/
  bottom), so pocket corners touching two or more sides beat single-side
  spots; ties break by in-group membership, then (y, x). Failure falls back
  to `_place_at_boundary()`.

## Step 7 — Design merge

`_step7_design_merge()`

- For every cross-group D2D connection, compute the translation that brings
  the target group to the source group: the dies abut with
  `_group_clearance()` (max margin across both groups + epsilon) and the PHYs
  align, following the PHY-edge combination (right–left, top–bottom, or
  same-axis edges with the current relative position).
- Multiple shifts onto the same target group are averaged; the whole group
  (dominant + slaves + isolated) is translated together.

## Overlap / spacing resolution

`_resolve_overlaps_and_spacing()` — after merge, up to 100 iterations.

- **Only free isolated instances move.** Anchored instances (dominants and
  D2D-connected slaves) keep their exact abutment/PHY alignment; LSI bridges
  keep their H7 midpoint; base-layer instances are excluded entirely.
- Whenever two margin-inflated AABBs on overlapping Z layers intersect (D2D
  pairs excepted), the movable side is shifted apart by
  `GeometryEngine.resolve_overlap()` (both movable → split the correction;
  single movable → doubled).

## Step 8 — Merged design placement

`_step8_center_design()`

- Translate the whole design so the instance MBR center coincides with the
  base-layer center (actual base instance AABB center → base def size/2 →
  shift into positive enclosure coordinates). Base instances never move.

## Finalization

`solve()` concludes with:

1. `ConstraintChecker.check_all()` — the H1–H7 hard-rule and S1–S5 soft-rule
   report.
2. Interposer size = instance MBR + 2 × enclosure.
3. `PlacementSolution` (design, poses, interposer size, score, report).

## Verified results

| Case | Result |
|------|--------|
| CoWoS_S | Valid, score 0.9333 |
| CoWoS_L | Valid, score 0.9666 |
| All-In-One (CoW) | Valid, score 0.9358 |

Exported `.3dbx` files re-parse cleanly and reproduce the reported score.
