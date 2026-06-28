"""
Visualization module for 3D IC Chiplets Coarse-Placement System.

Generates floorplan images, score tables, and statistics.
"""

import math
import os
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, Rectangle


def _get_color(ref_name: str) -> str:
    """Return a color for a chiplet reference type."""
    ref_upper = ref_name.upper()
    if "INTERPOSER" in ref_upper:
        return "#E0E0E0"
    if "SOC" in ref_upper or "SOIC" in ref_upper:
        return "#4A90D9"
    if "HBM" in ref_upper or "MEM" in ref_upper:
        return "#5DAD5D"
    if "LSI" in ref_upper:
        return "#F5A623"
    if "IOD" in ref_upper:
        return "#9B59B6"
    if "DUMMY" in ref_upper:
        return "#BDC3C7"
    return "#E74C3C"


def _get_edge_color(ref_name: str) -> str:
    """Return edge color for a chiplet reference type."""
    ref_upper = ref_name.upper()
    if "INTERPOSER" in ref_upper:
        return "#95A5A6"
    if "LSI" in ref_upper:
        return "#D35400"
    return "#2C3E50"


def plot_floorplan(design, output_path: str, title: str = "Floorplan", dpi: int = 150) -> None:
    """Generate a 2D floorplan image of the placement.
    
    Args:
        design: The DesignModel with final placement poses.
        output_path: Path to save the PNG image.
        title: Plot title.
        dpi: Image resolution.
    """
    fig, ax = plt.subplots(figsize=(14, 10))
    
    # Determine overall bounds
    bboxes = []
    for inst in design.instances:
        def_ = design.get_def(inst.reference)
        if def_:
            bboxes.append(inst.global_aabb(def_))
    
    if not bboxes:
        ax.set_title("No instances to display")
        plt.savefig(output_path, dpi=dpi, bbox_inches="tight")
        plt.close()
        return
    
    min_x = min(b.x1 for b in bboxes)
    min_y = min(b.y1 for b in bboxes)
    max_x = max(b.x2 for b in bboxes)
    max_y = max(b.y2 for b in bboxes)
    
    margin_x = (max_x - min_x) * 0.05 + 500
    margin_y = (max_y - min_y) * 0.05 + 500
    
    ax.set_xlim(min_x - margin_x, max_x + margin_x)
    ax.set_ylim(min_y - margin_y, max_y + margin_y)
    ax.set_aspect("equal")
    ax.set_xlabel("X (μm)")
    ax.set_ylabel("Y (μm)")
    ax.set_title(title)
    
    # Draw instances
    legend_handles = {}
    for inst in design.instances:
        def_ = design.get_def(inst.reference)
        if not def_:
            continue
        aabb = inst.global_aabb(def_)
        color = _get_color(inst.reference)
        edge_color = _get_edge_color(inst.reference)
        
        # Interposer goes behind as a filled rectangle
        if "INTERPOSER" in inst.reference.upper():
            rect = Rectangle((aabb.x1, aabb.y1), aabb.width, aabb.height,
                             facecolor=color, edgecolor=edge_color, linewidth=2, zorder=0)
            ax.add_patch(rect)
            legend_handles["Interposer"] = mpatches.Patch(color=color, label="Interposer")
            continue
        
        rect = FancyBboxPatch((aabb.x1, aabb.y1), aabb.width, aabb.height,
                              boxstyle="square,pad=0",
                              facecolor=color, edgecolor=edge_color,
                              linewidth=1.5, alpha=0.85, zorder=2)
        ax.add_patch(rect)
        
        # Label in center
        cx, cy = aabb.center
        ax.text(cx, cy, inst.name, ha="center", va="center",
                fontsize=7, color="white" if color in ("#4A90D9", "#9B59B6", "#D35400") else "black",
                fontweight="bold", zorder=3)
        
        # Legend tracking
        label = None
        if "SOC" in inst.reference.upper() or "SOIC" in inst.reference.upper():
            label = "SOC/SoIC"
        elif "HBM" in inst.reference.upper() or "MEM" in inst.reference.upper():
            label = "HBM/MEM"
        elif "LSI" in inst.reference.upper():
            label = "LSI"
        elif "IOD" in inst.reference.upper():
            label = "IOD"
        elif "DUMMY" in inst.reference.upper():
            label = "Dummy"
        if label and label not in legend_handles:
            legend_handles[label] = mpatches.Patch(color=color, label=label)
        
        # Draw IPs as small dots
        for entry in def_.omap_entries:
            obj_size = def_.get_object_size(entry.obj_type)
            local_cx = entry.loc_x + obj_size[0] / 2.0
            local_cy = entry.loc_y + obj_size[1] / 2.0
            from .core.geometry import GeometryEngine
            gx, gy = GeometryEngine.local_to_global(inst.pose, local_cx, local_cy, def_.width, def_.height)
            ax.plot(gx, gy, "o", color="red", markersize=2.5, zorder=4)
    
    # Draw D2D connections
    for conn in design.d2d_connections:
        positions = design.get_d2d_ip_positions(conn)
        if not positions:
            continue
        (sx, sy), (tx, ty) = positions
        if conn.has_lsi:
            lsi_inst = design.get_instance(conn.lsi_inst)
            if lsi_inst:
                lsi_def = design.get_def(lsi_inst.reference)
                if lsi_def:
                    lsi_aabb = lsi_inst.global_aabb(lsi_def)
                    lsi_cx, lsi_cy = lsi_aabb.center
                    ax.plot([sx, lsi_cx, tx], [sy, lsi_cy, ty], "-", color="#E67E22", linewidth=1.5, zorder=1)
                    ax.plot(lsi_cx, lsi_cy, "s", color="#E67E22", markersize=5, zorder=3)
            else:
                ax.plot([sx, tx], [sy, ty], "--", color="#E67E22", linewidth=1.5, zorder=1)
        else:
            ax.plot([sx, tx], [sy, ty], "-", color="#C0392B", linewidth=1.5, zorder=1)
        ax.plot(sx, sy, "o", color="#C0392B", markersize=3, zorder=4)
        ax.plot(tx, ty, "o", color="#C0392B", markersize=3, zorder=4)
    
    if legend_handles:
        ax.legend(handles=list(legend_handles.values()), loc="upper right", fontsize=8)
    
    ax.grid(True, alpha=0.3, linestyle="--")
    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close()


def export_score_json(report, output_path: str) -> None:
    """Export the score report as a JSON file."""
    import json
    data = {
        "is_valid": report.is_valid,
        "total_score": report.total_score,
        "hard_violations": report.hard_violations,
        "soft_scores": {k: float(v) for k, v in report.soft_scores.items()},
        "score_details": report.score_details,
    }
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def export_score_csv(report, output_path: str) -> None:
    """Export the score report as a CSV file."""
    import csv
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Category", "Metric", "Value"])
        writer.writerow(["Overall", "Valid", "Yes" if report.is_valid else "No"])
        writer.writerow(["Overall", "Total Score", f"{report.total_score:.6f}"])
        writer.writerow([])
        writer.writerow(["Hard Violations", "Count", len(report.hard_violations)])
        for i, v in enumerate(report.hard_violations, 1):
            writer.writerow(["Hard Violations", f"Violation {i}", v])
        writer.writerow([])
        writer.writerow(["Soft Scores", "Rule", "Score"])
        for rule, score in report.soft_scores.items():
            writer.writerow(["Soft Scores", rule, f"{score:.6f}"])
        writer.writerow([])
        writer.writerow(["Weights", "Rule", "Weight"])
        # weights are not in report, handled by caller


def plot_score_table(report, output_path: str, dpi: int = 150) -> None:
    """Generate a score table image (matplotlib table)."""
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axis("off")
    
    rows = [
        ["Metric", "Value"],
        ["Valid", "Yes" if report.is_valid else "No"],
        ["Total Score", f"{report.total_score:.4f}"],
        ["", ""],
        ["Soft Rule", "Score"],
    ]
    for rule, score in report.soft_scores.items():
        rows.append([rule, f"{score:.4f}"])
    
    if report.hard_violations:
        rows.append(["", ""])
        rows.append(["Hard Violations", ""])
        for v in report.hard_violations:
            rows.append(["", v])
    
    table = ax.table(cellText=rows, cellLoc="left", loc="center",
                     colWidths=[0.5, 0.5])
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)
    
    # Header style
    for i in range(2):
        table[(0, i)].set_facecolor("#3498DB")
        table[(0, i)].set_text_props(color="white", fontweight="bold")
    
    # Sub-header style
    for j in [4]:
        for i in range(2):
            table[(j, i)].set_facecolor("#ECF0F1")
            table[(j, i)].set_text_props(fontweight="bold")
    
    plt.title("Placement Score Report", fontsize=14, fontweight="bold", pad=20)
    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close()
