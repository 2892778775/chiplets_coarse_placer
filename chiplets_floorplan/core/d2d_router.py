"""
D2D Router: PHY alignment and abutment refinement.

Given a DesignModel with D2D connections, this module:
1. Computes global positions of all involved PHYs.
2. Tries all legal flip/rotate combinations for involved instances.
3. Picks the combination that minimizes total D2D PHY distance.
4. Optionally generates LSI (Local Silicon Interconnect) chiplets if PHYs cannot directly abut.
"""

import itertools
import math
from typing import List, Tuple, Optional, Dict
from .models import DesignModel, ChipletInst, ChipletDef, InstancePose, D2DConnection, ObjectMapEntry, AABB
from .geometry import GeometryEngine


class D2DRouter:
    """Refine D2D PHY alignment and abutment."""

    def __init__(self, design: DesignModel, alignment_tol: float = 10.0, abutment_tol: float = 10.0):
        self.design = design
        self.alignment_tol = alignment_tol  # micron tolerance for PHY center alignment
        self.abutment_tol = abutment_tol    # micron tolerance for abutment

    def refine(self, max_iterations: int = 50) -> int:
        """
        Try all legal flip/rotate combinations for instances involved in D2D connections.
        Pick the combination that minimizes total D2D PHY distance.
        Returns the number of connections that remain unaligned (0 = all aligned).
        """
        if not self.design.d2d_connections:
            return 0

        # Collect all instances involved in direct (non-LSI) D2D connections
        involved_names = set()
        direct_connections = [c for c in self.design.d2d_connections if not c.has_lsi]
        for conn in direct_connections:
            involved_names.add(conn.source_inst)
            involved_names.add(conn.target_inst)

        involved_instances = [self.design.get_instance(name) for name in involved_names]
        involved_instances = [inst for inst in involved_instances if inst is not None]

        if not involved_instances or not direct_connections:
            return 0

        # Save original poses
        original_poses = {inst.name: inst.pose.copy() for inst in involved_instances}

        # Generate flip options
        flip_options = ["None", "MX", "MY"]
        orient_options = ["R0", "R90", "R180", "R270"]

        best_score = float('inf')
        best_config = None

        # For each involved instance, try all combinations of flip and orientation
        # We limit to flip-only for now, since orientation changes are more complex geometrically
        flip_combinations = list(itertools.product(flip_options, repeat=len(involved_instances)))

        for flip_combo in flip_combinations:
            # Apply flip combination
            for i, inst in enumerate(involved_instances):
                inst.pose.flip = flip_combo[i]

            # Compute total D2D distance for direct connections only
            total_dist = 0.0
            valid_count = 0
            for conn in direct_connections:
                positions = self.design.get_d2d_ip_positions(conn)
                if positions:
                    (sx, sy), (tx, ty) = positions
                    total_dist += math.hypot(sx - tx, sy - ty)
                    valid_count += 1

            # Also compute alignment penalty for direct connections only
            alignment_penalty = 0.0
            for conn in direct_connections:
                positions = self.design.get_d2d_ip_positions(conn)
                if positions:
                    (sx, sy), (tx, ty) = positions
                    if abs(sx - tx) > self.alignment_tol and abs(sy - ty) > self.alignment_tol:
                        alignment_penalty += 10000.0  # Heavy penalty for misalignment

            score = total_dist + alignment_penalty
            if score < best_score:
                best_score = score
                best_config = flip_combo

        # Restore best configuration
        if best_config is not None:
            for i, inst in enumerate(involved_instances):
                inst.pose.flip = best_config[i]

        # Now, try to align and abut each direct connection by fine-shifting instances
        unaligned = self._align_and_abut_connections(max_iterations)

        return unaligned

    def _align_and_abut_connections(self, max_iterations: int = 50) -> int:
        """
        For each D2D connection, try to align and abut the PHYs by shifting the target instance.
        Returns number of connections that could not be fully aligned/abutted.
        """
        unaligned = 0
        for conn in self.design.d2d_connections:
            if conn.has_lsi:
                # Skip LSI-bridged connections for now
                continue
            src_inst = self.design.get_instance(conn.source_inst)
            tgt_inst = self.design.get_instance(conn.target_inst)
            if not src_inst or not tgt_inst:
                unaligned += 1
                continue

            src_def = self.design.get_def(src_inst.reference)
            tgt_def = self.design.get_def(tgt_inst.reference)
            if not src_def or not tgt_def:
                unaligned += 1
                continue

            src_ip_pos = src_inst.global_ip_position(src_def, conn.source_ip)
            tgt_ip_pos = tgt_inst.global_ip_position(tgt_def, conn.target_ip)
            if not src_ip_pos or not tgt_ip_pos:
                unaligned += 1
                continue

            src_ip_aabb = src_inst.global_ip_aabb(src_def, conn.source_ip)
            tgt_ip_aabb = tgt_inst.global_ip_aabb(tgt_def, conn.target_ip)
            if not src_ip_aabb or not tgt_ip_aabb:
                unaligned += 1
                continue

            # Check if already aligned and abutting
            if self._is_aligned_and_abutting(src_ip_aabb, tgt_ip_aabb):
                continue

            # Try to shift target to align and abut
            aligned = self._shift_target_to_abut(src_inst, tgt_inst, src_def, tgt_def, 
                                                  conn.source_ip, conn.target_ip)
            if not aligned:
                unaligned += 1

        return unaligned

    def _is_aligned_and_abutting(self, aabb1: AABB, aabb2: AABB) -> bool:
        """Check if two PHY AABBs are aligned and abutting."""
        # Check alignment
        cx1, cy1 = aabb1.center
        cx2, cy2 = aabb2.center
        aligned_x = abs(cx1 - cx2) < self.alignment_tol
        aligned_y = abs(cy1 - cy2) < self.alignment_tol
        aligned = aligned_x or aligned_y
        if not aligned:
            return False

        # Check abutment
        abutting = GeometryEngine.check_abutment(aabb1, aabb2, self.abutment_tol)
        return abutting

    def _shift_target_to_abut(self, src_inst: ChipletInst, tgt_inst: ChipletInst,
                               src_def: ChipletDef, tgt_def: ChipletDef,
                               src_ip_name: str, tgt_ip_name: str) -> bool:
        """
        Try to shift the target instance so that its PHY aligns and abuts with the source PHY.
        Returns True if successful.
        """
        src_ip_pos = src_inst.global_ip_position(src_def, src_ip_name)
        tgt_ip_pos = tgt_inst.global_ip_position(tgt_def, tgt_ip_name)
        if not src_ip_pos or not tgt_ip_pos:
            return False

        src_ip_aabb = src_inst.global_ip_aabb(src_def, src_ip_name)
        tgt_ip_aabb = tgt_inst.global_ip_aabb(tgt_def, tgt_ip_name)
        if not src_ip_aabb or not tgt_ip_aabb:
            return False

        # Determine preferred abutment direction based on relative positions
        src_cx, src_cy = src_ip_aabb.center
        tgt_cx, tgt_cy = tgt_ip_aabb.center

        dx = tgt_cx - src_cx
        dy = tgt_cy - src_cy

        # Try shifting in the dominant direction
        if abs(dx) >= abs(dy):
            # Try horizontal abutment
            if dx > 0:
                # Target is to the right, shift left to abut
                desired_tgt_x = src_ip_aabb.x2 - (tgt_ip_aabb.center[0] - tgt_inst.pose.x) + tgt_ip_aabb.width / 2
                new_x = src_ip_aabb.x2 - (tgt_ip_aabb.x2 - tgt_inst.pose.x)
            else:
                # Target is to the left, shift right to abut
                new_x = src_ip_aabb.x1 - (tgt_ip_aabb.x1 - tgt_inst.pose.x)
            
            tgt_inst.pose.x = new_x
            # Re-align Y centers
            tgt_inst.pose.y += src_cy - tgt_ip_aabb.center[1]
        else:
            # Try vertical abutment
            if dy > 0:
                # Target is above, shift down to abut
                new_y = src_ip_aabb.y2 - (tgt_ip_aabb.y2 - tgt_inst.pose.y)
            else:
                # Target is below, shift up to abut
                new_y = src_ip_aabb.y1 - (tgt_ip_aabb.y1 - tgt_inst.pose.y)
            
            tgt_inst.pose.y = new_y
            # Re-align X centers
            tgt_inst.pose.x += src_cx - tgt_ip_aabb.center[0]

        # Verify
        tgt_ip_aabb_new = tgt_inst.global_ip_aabb(tgt_def, tgt_ip_name)
        if tgt_ip_aabb_new and self._is_aligned_and_abutting(src_ip_aabb, tgt_ip_aabb_new):
            return True

        return False

    def generate_lsi(self, conn: D2DConnection, lsi_name: str = "LSI") -> Optional[ChipletDef]:
        """
        Generate an LSI (Local Silicon Interconnect) chiplet definition that bridges two PHYs.
        This is called when direct abutment is impossible or when user requests LSI.
        Returns the LSI ChipletDef or None if positions cannot be determined.
        """
        src_inst = self.design.get_instance(conn.source_inst)
        tgt_inst = self.design.get_instance(conn.target_inst)
        if not src_inst or not tgt_inst:
            return None

        src_def = self.design.get_def(src_inst.reference)
        tgt_def = self.design.get_def(tgt_inst.reference)
        if not src_def or not tgt_def:
            return None

        src_ip_pos = src_inst.global_ip_position(src_def, conn.source_ip)
        tgt_ip_pos = tgt_inst.global_ip_position(tgt_def, conn.target_ip)
        if not src_ip_pos or not tgt_ip_pos:
            return None

        # LSI bounds: MBR of the two PHY positions + small padding
        min_x = min(src_ip_pos[0], tgt_ip_pos[0])
        min_y = min(src_ip_pos[1], tgt_ip_pos[1])
        max_x = max(src_ip_pos[0], tgt_ip_pos[0])
        max_y = max(src_ip_pos[1], tgt_ip_pos[1])

        # Get IP sizes for padding
        src_ip_aabb = src_inst.global_ip_aabb(src_def, conn.source_ip)
        tgt_ip_aabb = tgt_inst.global_ip_aabb(tgt_def, conn.target_ip)
        if src_ip_aabb and tgt_ip_aabb:
            min_x = min(src_ip_aabb.x1, tgt_ip_aabb.x1)
            min_y = min(src_ip_aabb.y1, tgt_ip_aabb.y1)
            max_x = max(src_ip_aabb.x2, tgt_ip_aabb.x2)
            max_y = max(src_ip_aabb.y2, tgt_ip_aabb.y2)

        width = max_x - min_x
        height = max_y - min_y
        if width <= 0 or height <= 0:
            return None

        lsi_def = ChipletDef(
            name=lsi_name,
            size=(width, height),
            shrink=1.0,
            thickness=150,
            seal_ring=[18, 18, 18, 18],
            scribe_line=[76, 76, 76, 76]
        )
        return lsi_def
