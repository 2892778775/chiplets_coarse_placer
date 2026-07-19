#!/usr/bin/env python3
"""
3D IC Chiplets Coarse-Placement System — CLI Automation

Usage:
    python run_cli.py --3dbx CoWoS_S/CoWoS-S.3dbx --connection CoWoS_S/D2D.connection --placer expert --output output/
    python run_cli.py --dbx CoWoS_L/CoWoS-L.3dbx --connection CoWoS_L/D2D.connection --algorithm SA --output out_l/

Outputs:
    - 3Dblox files (.3dbx, .3dbv, .3dbo, .omap)
    - floorplan.png   (2D placement visualization)
    - score_table.png (score report image)
    - score.json      (machine-readable scores)
    - score.csv       (spreadsheet-compatible scores)
"""

import sys
import os
import argparse
import json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from chiplets_floorplan.core.parser import Parser
from chiplets_floorplan.core.placer import Placer
from chiplets_floorplan.core.d2d_router import D2DRouter
from chiplets_floorplan.core.compaction import Compaction
from chiplets_floorplan.core.exporter import Exporter
from chiplets_floorplan.core.constraints import ConstraintChecker
from chiplets_floorplan.viz import (
    plot_floorplan, plot_score_table, export_score_json, export_score_csv
)


def main():
    parser = argparse.ArgumentParser(
        description="3D IC Chiplets Coarse-Placement System — CLI Automation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Expert placement (default)
  python run_cli.py --dbx CoWoS_S/CoWoS-S.3dbx --connection CoWoS_S/D2D.connection --output output/

  # Simulated Annealing placement
  python run_cli.py --dbx CoWoS_L/CoWoS-L.3dbx --connection CoWoS_L/D2D.connection --output out_l/ --algorithm SA

  # Skip D2D refinement
  python run_cli.py --dbx design.3dbx --output output/ --skip-d2d
        """
    )
    parser.add_argument("--dbx", "--3dbx", dest="dbx", required=True,
                        help="Path to the top-level .3dbx input file")
    parser.add_argument("--connection", default="", help="Path to D2D connection file (.connection)")
    parser.add_argument("--output", default="output", help="Output directory for all artifacts")
    parser.add_argument("--algorithm", "--placer", dest="algorithm", choices=["SA", "Expert", "sa", "expert"],
                        default="Expert", help="Placement algorithm: SA (Simulated Annealing) or Expert (rule-based)")
    parser.add_argument("--sa-iterations", type=int, default=5000, help="SA iterations (only for SA algorithm)")
    parser.add_argument("--enclosure", type=float, default=500.0, help="Minimum interposer enclosure (um)")
    parser.add_argument("--dpi", type=int, default=150, help="Image resolution DPI")
    parser.add_argument("--skip-d2d", action="store_true", help="Skip D2D refinement")
    parser.add_argument("--no-images", action="store_true", help="Skip PNG image generation (floorplan + score table)")
    parser.add_argument("--no-json", action="store_true", help="Skip score.json output")
    parser.add_argument("--no-csv", action="store_true", help="Skip score.csv output")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for SA algorithm reproducibility")
    parser.add_argument("--quiet", action="store_true", help="Suppress non-essential console output")
    args = parser.parse_args()
    # Normalize algorithm name (accept expert/sa in any case)
    args.algorithm = "SA" if args.algorithm.upper() == "SA" else "Expert"

    def log(msg):
        if not args.quiet:
            print(msg)

    import time
    start_time = time.time()

    log("=" * 60)
    log("3D IC Chiplets Coarse-Placement System — CLI")
    log(f"Input:      {args.dbx}")
    log(f"Algorithm:  {args.algorithm}")
    log(f"Output:     {args.output}")
    log("=" * 60)

    os.makedirs(args.output, exist_ok=True)

    # ------------------------------------------------------------------
    # 1. Parse input
    # ------------------------------------------------------------------
    log("\n[1/5] Parsing 3Dblox input...")
    if not os.path.exists(args.dbx):
        log(f"ERROR: .3dbx file not found: {args.dbx}")
        return 2

    try:
        parser_obj = Parser()
        design = parser_obj.parse_design(args.dbx)
    except Exception as e:
        log(f"ERROR: Failed to parse design: {e}")
        return 2

    log(f"  Design:    {design.name}")
    log(f"  Chiplets:  {len(design.chiplet_defs)}")
    log(f"  Instances: {len(design.instances)}")

    # Parse D2D connections if file provided
    if args.connection:
        if not os.path.exists(args.connection):
            log(f"  WARNING: Connection file not found: {args.connection}")
        else:
            with open(args.connection, "r", encoding="utf-8") as f:
                design.d2d_connections = parser_obj.parse_connections(f.read())
            log(f"  D2D connections: {len(design.d2d_connections)}")
    else:
        design.d2d_connections = []

    # Set random seed if provided
    if args.seed is not None:
        import random
        random.seed(args.seed)
        log(f"  Random seed: {args.seed}")

    # ------------------------------------------------------------------
    # 2. Placement
    # ------------------------------------------------------------------
    log(f"\n[2/5] Running placement ({args.algorithm})...")
    if args.algorithm == "SA":
        placer = Placer(design, algorithm="SA", sa_iterations=args.sa_iterations, enclosure=args.enclosure)
    else:
        placer = Placer(design, algorithm="Expert", enclosure=args.enclosure)
    solution = placer.solve()
    log(f"  Score: {solution.score:.4f}")
    log(f"  Valid: {solution.report.is_valid}")
    if not solution.report.is_valid:
        log("  Hard violations:")
        for v in solution.report.hard_violations:
            log(f"    - {v}")

    # ------------------------------------------------------------------
    # 3. D2D refinement
    # ------------------------------------------------------------------
    if not args.skip_d2d and design.d2d_connections:
        log("\n[3/5] Refining D2D PHY alignment...")
        pre_refine_score = solution.score
        pre_refine_report = solution.report
        pre_refine_poses = {inst.name: inst.pose.copy() for inst in design.instances}

        router = D2DRouter(design)
        unaligned = router.refine()
        log(f"  Unaligned connections: {unaligned}")

        checker = ConstraintChecker(design)
        report = checker.check_all()
        solution.score = report.total_score
        solution.report = report
        log(f"  Score after D2D: {solution.score:.4f}")

        # If D2D refinement broke the placement, revert to pre-refinement state
        if not report.is_valid or solution.score < pre_refine_score:
            log(f"  WARNING: D2D refinement degraded placement (valid={report.is_valid}, score={solution.score:.4f}). Reverting...")
            for inst in design.instances:
                if inst.name in pre_refine_poses:
                    inst.pose = pre_refine_poses[inst.name].copy()
            solution.score = pre_refine_score
            solution.report = pre_refine_report
    else:
        log("\n[3/5] Skipping D2D refinement.")

    # ------------------------------------------------------------------
    # 4. Compaction
    # ------------------------------------------------------------------
    log("\n[4/5] Compaction and interposer sizing...")
    compactor = Compaction(design, min_enclosure=args.enclosure)
    compactor.update_interposer()
    w, h = compactor.compute_interposer_size()
    solution.interposer_size = (w, h)
    log(f"  Interposer size: {w:.0f} x {h:.0f} um")

    # Re-check after the interposer update: the enclosure rule (H6) depends
    # on the final base-layer geometry, so the SUMMARY must reflect a fresh
    # check rather than the pre-compaction report.
    checker = ConstraintChecker(design)
    report = checker.check_all()
    solution.score = report.total_score
    solution.report = report
    log(f"  Score after compaction: {solution.score:.4f} (valid: {report.is_valid})")

    # ------------------------------------------------------------------
    # 6. Export artifacts
    # ------------------------------------------------------------------
    log("\n[5/5] Exporting artifacts...")

    # 6a. 3Dblox files
    design_name = design.name or os.path.splitext(os.path.basename(args.dbx))[0]
    exporter = Exporter(solution)
    blox_files = exporter.export(args.output, design_name)
    log(f"  3Dblox files: {len(blox_files)}")
    for f in blox_files:
        log(f"    {os.path.basename(f)}")

    # 6b. Floorplan image
    if not args.no_images:
        floorplan_path = os.path.join(args.output, "floorplan.png")
        plot_floorplan(design, floorplan_path,
                       title=f"{design.name} — {args.algorithm} Placement",
                       dpi=args.dpi)
        log(f"  Floorplan: {floorplan_path}")

        # 6c. Score table image
        score_table_path = os.path.join(args.output, "score_table.png")
        plot_score_table(solution.report, score_table_path, dpi=args.dpi)
        log(f"  Score table: {score_table_path}")
    else:
        log("  Skipping image generation (--no-images)")

    # 6d. Score JSON
    if not args.no_json:
        score_json_path = os.path.join(args.output, "score.json")
        export_score_json(solution.report, score_json_path)
        log(f"  Score JSON: {score_json_path}")
    else:
        log("  Skipping score.json (--no-json)")

    # 6e. Score CSV
    if not args.no_csv:
        score_csv_path = os.path.join(args.output, "score.csv")
        export_score_csv(solution.report, score_csv_path)
        # Append weights to CSV
        checker = ConstraintChecker(design)
        with open(score_csv_path, "a", newline="", encoding="utf-8") as f:
            import csv
            writer = csv.writer(f)
            writer.writerow([])
            writer.writerow(["Weights", "Rule", "Weight"])
            for rule, weight in checker.weights.items():
                writer.writerow(["Weights", rule, f"{weight:.6f}"])
        log(f"  Score CSV: {score_csv_path}")
    else:
        log("  Skipping score.csv (--no-csv)")

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    elapsed = time.time() - start_time
    log("\n" + "=" * 60)
    log("SUMMARY")
    log(f"  Valid:          {solution.report.is_valid}")
    log(f"  Total Score:    {solution.score:.4f}")
    for rule, score in solution.report.soft_scores.items():
        log(f"  {rule:20s}: {score:.4f}")
    log(f"  Interposer:     {solution.interposer_size[0]:.0f} x {solution.interposer_size[1]:.0f} um")
    log(f"  Output dir:     {os.path.abspath(args.output)}")
    log(f"  Elapsed time:   {elapsed:.2f}s")
    log("=" * 60)

    return 0 if solution.report.is_valid else 1


if __name__ == "__main__":
    sys.exit(main())
