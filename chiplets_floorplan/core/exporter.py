"""
3Dblox Exporter: write a PlacementSolution back to standard 3Dblox format.
"""

import os

# Only needed if yaml is available; exporter writes plain text
# try:
#     import yaml
# except ImportError:
#     pass

from typing import Optional
from .models import PlacementSolution, DesignModel, ChipletInst, ChipletDef


class Exporter:
    """Export a PlacementSolution to 3Dblox files."""

    @staticmethod
    def _fmt_coord(value: float) -> str:
        """Format a coordinate, preserving sub-micron precision when needed.

        Integer truncation would shift LSI centers off the exact D2D PHY
        midpoint (hard rule H7, tolerance 1e-6), so keep up to 4 decimals.
        """
        if abs(value - round(value)) < 1e-9:
            return str(int(round(value)))
        return f"{value:.4f}".rstrip("0").rstrip(".")

    def __init__(self, solution: PlacementSolution):
        self.solution = solution
        self.design = solution.design

    def export(self, output_dir: str, design_name: Optional[str] = None) -> list:
        """
        Export the placement solution to 3Dblox files.
        Returns a list of exported file paths.
        """
        os.makedirs(output_dir, exist_ok=True)
        name = design_name or self.design.name or "design"
        exported_files = []

        # 1. Export main .3dbx file
        dbx_path = os.path.join(output_dir, f"{name}_export.3dbx")
        self._write_3dbx(dbx_path, name)
        exported_files.append(dbx_path)

        # 2. Export main .3dbv file (includes all chiplet .3dbv files)
        dbv_path = os.path.join(output_dir, f"{name}_export.3dbv")
        self._write_main_3dbv(dbv_path, name)
        exported_files.append(dbv_path)

        # 3. Export individual chiplet .3dbv, .3dbo, .omap files.
        # Dummy defs must be exported as well: dummy instances are listed in
        # the main .3dbx, so their references must resolve on re-parse.
        for chiplet_name, chiplet_def in self.design.chiplet_defs.items():
            chiplet_dbv = os.path.join(output_dir, f"{chiplet_name}.3dbv")
            self._write_chiplet_3dbv(chiplet_dbv, chiplet_def)
            exported_files.append(chiplet_dbv)

            if chiplet_def.object_defs:
                chiplet_dbo = os.path.join(output_dir, f"{chiplet_name}.3dbo")
                self._write_chiplet_3dbo(chiplet_dbo, chiplet_def)
                exported_files.append(chiplet_dbo)

            if chiplet_def.omap_entries:
                chiplet_omap = os.path.join(output_dir, f"{chiplet_name}.omap")
                self._write_chiplet_omap(chiplet_omap, chiplet_def)
                exported_files.append(chiplet_omap)

        return exported_files

    def _write_3dbx(self, path: str, name: str) -> None:
        """Write the top-level .3dbx file."""
        with open(path, "w", encoding="utf-8") as f:
            f.write("Header:\n")
            f.write("  version: 3.0\n")
            f.write("  unit: micron\n")
            f.write("  precision: 10000\n")
            f.write("  include:\n")
            f.write(f"    - {name}_export.3dbv\n")
            f.write("Design:\n")
            f.write(f"  name: {name.replace('-', '_')}\n")
            f.write("ChipletInst:\n")
            
            for inst in self.design.instances:
                f.write(f"  {inst.name}:\n")
                f.write(f"    reference: {inst.reference}\n")
                f.write(f"    is_master: {str(inst.is_master).lower()}\n")
            
            f.write("\nStack:\n")
            for inst in self.design.instances:
                f.write(f"  {inst.name}:\n")
                f.write(f"    loc: [{self._fmt_coord(inst.pose.x)}, {self._fmt_coord(inst.pose.y)}]\n")
                f.write(f"    z: {self._fmt_coord(inst.pose.z)}\n")
                # Combine flip, mz, and orientation into a single orient value
                # Examples: MX_MZ_R90, MY_R0, MZ_R180, R0
                parts = []
                if inst.pose.flip and inst.pose.flip != "None":
                    parts.append(inst.pose.flip)
                if inst.pose.mz:
                    parts.append("MZ")
                if inst.pose.orientation:
                    parts.append(inst.pose.orientation)
                f.write(f"    orient: {'_'.join(parts)}\n")

    def _write_main_3dbv(self, path: str, name: str) -> None:
        """Write the main .3dbv file with include references."""
        with open(path, "w", encoding="utf-8") as f:
            f.write("Header:\n")
            f.write("  version: 3.0\n")
            f.write("  unit: micron\n")
            f.write("  precision: 1000\n")
            f.write("  include:\n")
            for chiplet_name in self.design.chiplet_defs:
                f.write(f"  - {chiplet_name}.3dbv\n")

    def _write_chiplet_3dbv(self, path: str, chiplet_def: ChipletDef) -> None:
        """Write a single chiplet .3dbv file."""
        with open(path, "w", encoding="utf-8") as f:
            f.write("Header:\n")
            f.write("  version: 3.0\n")
            f.write("  unit: micron\n")
            f.write("  precision: 10000\n")
            f.write("ChipletDef:\n")
            f.write(f"  {chiplet_def.name}:\n")
            f.write(f"    size: {list(chiplet_def.size)}\n")
            f.write(f"    shrink: {chiplet_def.shrink}\n")
            f.write(f"    thickness: {chiplet_def.thickness}\n")
            f.write(f"    seal_ring: {chiplet_def.seal_ring}\n")
            f.write(f"    scribe_line_remaining_width: {chiplet_def.scribe_line}\n")
            if chiplet_def.omap_entries:
                f.write(f"    omap: .\\{chiplet_def.name}.omap\n")
            if chiplet_def.object_defs:
                f.write("    external:\n")
                f.write(f"      3dbo_file:[.\\{chiplet_def.name}.3dbo]\n")

    def _write_chiplet_3dbo(self, path: str, chiplet_def: ChipletDef) -> None:
        """Write a .3dbo file."""
        with open(path, "w", encoding="utf-8") as f:
            f.write("ObjectDef:\n")
            for obj_name, obj_def in chiplet_def.object_defs.items():
                f.write(f"  {obj_name}:\n")
                f.write(f"    size: {list(obj_def.size)}\n")
                if obj_def.layer:
                    f.write(f"    layer: {obj_def.layer}\n")

    def _write_chiplet_omap(self, path: str, chiplet_def: ChipletDef) -> None:
        """Write an .omap file."""
        with open(path, "w", encoding="utf-8") as f:
            for entry in chiplet_def.omap_entries:
                f.write(f"{entry.obj_type} {entry.name} {int(entry.loc_x)} {int(entry.loc_y)} {entry.orientation}\n")
