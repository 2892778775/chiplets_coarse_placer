"""
Constraint system for the 3D IC Chiplets Coarse-Placement System.

Implements Hard Rules (must be satisfied) and Soft Rules (optimization targets).
All scores are normalized to [0, 1] range where higher is better.
"""

from typing import List, Dict, Tuple, Optional
from .models import DesignModel, ChipletInst, ViolationReport, InstancePose
from .geometry import GeometryEngine


# Constants
GEOMETRY_TOLERANCE = 1e-6
HARD_FAILURE_SCORE = -1e9
SPACING_OVERLAP_MARGIN = -1e-6
INTERPOSER_REF = "Interposer"
LSI_PREFIX = "LSI"

DEFAULT_WEIGHTS = {
    "vertical_symmetry": 0.15,
    "horizontal_symmetry": 0.15,
    "hbm_mem_placement": 0.20,
    "iod_placement": 0.20,
    "d2d_length_minimize": 0.30,
}

HBM_MEM_KEYWORDS = ("HBM", "MEM")
SOC_KEYWORDS = ("SOC", "SOIC")
IOD_KEYWORDS = ("IOD",)


def _is_lsi(inst: ChipletInst) -> bool:
    """Check if an instance is an LSI (bridge) chiplet."""
    return inst.reference.upper().startswith(LSI_PREFIX)


def _is_interposer(inst: ChipletInst) -> bool:
    """Check if an instance is the Interposer reference."""
    return inst.reference == INTERPOSER_REF


class ConstraintChecker:
    """Check all hard and soft rules against a DesignModel."""

    def __init__(self, design: DesignModel, weights: Optional[Dict[str, float]] = None):
        self.design = design
        self.weights = weights or DEFAULT_WEIGHTS.copy()
        self._base_ref = design.reference_def_name()

    def _is_base(self, inst: ChipletInst) -> bool:
        """Check if an instance is on the base/reference layer (Interposer or RW)."""
        return self._base_ref is not None and inst.reference == self._base_ref

    def check_all(self) -> ViolationReport:
        """Run all hard and soft rule checks, return a ViolationReport."""
        report = ViolationReport()
        
        # --- Hard Rules ---
        report.hard_violations.extend(self._check_no_overlap())
        report.hard_violations.extend(self._check_all_in_interposer())
        report.hard_violations.extend(self._check_d2d_alignment())
        report.hard_violations.extend(self._check_d2d_abutment())
        report.hard_violations.extend(self._check_min_spacing())
        report.hard_violations.extend(self._check_all_instances_centered())
        report.hard_violations.extend(self._check_lsi_alignment())
        
        # If any hard violation, score is -inf (or very low)
        if report.hard_violations:
            report.total_score = HARD_FAILURE_SCORE
            return report
        
        # --- Soft Rules ---
        rules = [
            ("vertical_symmetry", self._score_vertical_symmetry),
            ("horizontal_symmetry", self._score_horizontal_symmetry),
            ("hbm_mem_placement", self._score_hbm_mem_placement),
            ("iod_placement", self._score_iod_placement),
            ("d2d_length_minimize", self._score_d2d_length_minimize),
        ]
        
        total = 0.0
        for key, scorer in rules:
            score, detail = scorer()
            report.soft_scores[key] = score
            report.score_details[key] = detail
            w = self.weights.get(key, 0.0)
            total += w * score
        
        report.total_score = total
        return report

    # ------------------------------------------------------------------
    # Hard Rules
    # ------------------------------------------------------------------

    def _check_no_overlap(self) -> List[str]:
        """H1: No two chiplets may overlap in XY plane (only if on same Z layer)."""
        violations = []
        instances = [inst for inst in self.design.instances if not self._is_base(inst)]
        for i in range(len(instances)):
            def_i = self.design.get_def(instances[i].reference)
            if not def_i:
                continue
            aabb_i = instances[i].global_aabb(def_i)
            z_i_start = instances[i].pose.z
            z_i_end = z_i_start + def_i.thickness
            for j in range(i + 1, len(instances)):
                def_j = self.design.get_def(instances[j].reference)
                if not def_j:
                    continue
                # Only check overlap if chiplets occupy same Z layer
                z_j_start = instances[j].pose.z
                z_j_end = z_j_start + def_j.thickness
                if z_i_end <= z_j_start or z_j_end <= z_i_start:
                    continue
                aabb_j = instances[j].global_aabb(def_j)
                if aabb_i.overlaps(aabb_j):
                    violations.append(
                        f"H1: Overlap between {instances[i].name} and {instances[j].name}"
                    )
        return violations

    def _check_all_in_interposer(self) -> List[str]:
        """H2: All chiplets must be fully inside the Interposer boundary.
        
        Since interposer sizing is dynamic (Compaction computes MBR + enclosure),
        this check is automatically satisfied. Skip during placement.
        """
        return []

    def _check_d2d_alignment(self) -> List[str]:
        """H3: D2D PHY centers must be aligned on the same axis (X or Y).
        
        For LSI-bridged connections, alignment is relaxed (LSI acts as bridge).
        Tolerance is 1e-6 for coarse placement.
        """
        violations = []
        for conn in self.design.d2d_connections:
            if conn.has_lsi:
                continue
            positions = self.design.get_d2d_ip_positions(conn)
            if not positions:
                continue
            (sx, sy), (tx, ty) = positions
            aligned_x = abs(sx - tx) < GEOMETRY_TOLERANCE
            aligned_y = abs(sy - ty) < GEOMETRY_TOLERANCE
            if not aligned_x and not aligned_y:
                violations.append(
                    f"H3: D2D PHY misaligned: {conn.source_full} vs {conn.target_full}"
                )
        return violations

    def _check_d2d_abutment(self) -> List[str]:
        """H4: For directly connected D2D PHYs, the chiplet boundaries on the PHY sides must abut (space = 0).
        
        If PHY_A is on the right edge of chiplet A, and PHY_B is on the left edge of chiplet B,
        then chiplet A's right boundary must exactly touch chiplet B's left boundary (gap = 0).
        
        For LSI-bridged connections, abutment is relaxed (LSI acts as bridge).
        """
        violations = []
        for conn in self.design.d2d_connections:
            if conn.has_lsi:
                continue
            src_inst = self.design.get_instance(conn.source_inst)
            tgt_inst = self.design.get_instance(conn.target_inst)
            if not src_inst or not tgt_inst:
                continue
            src_def = self.design.get_def(src_inst.reference)
            tgt_def = self.design.get_def(tgt_inst.reference)
            if not src_def or not tgt_def:
                continue
            
            src_edge = self._get_phy_edge_global(src_inst, src_def, conn.source_ip)
            tgt_edge = self._get_phy_edge_global(tgt_inst, tgt_def, conn.target_ip)
            
            if not src_edge or not tgt_edge:
                continue
            
            src_aabb = src_inst.global_aabb(src_def)
            tgt_aabb = tgt_inst.global_aabb(tgt_def)
            
            abutting = False
            if src_edge == 'right' and tgt_edge == 'left':
                abutting = abs(src_aabb.x2 - tgt_aabb.x1) < GEOMETRY_TOLERANCE
            elif src_edge == 'left' and tgt_edge == 'right':
                abutting = abs(src_aabb.x1 - tgt_aabb.x2) < GEOMETRY_TOLERANCE
            elif src_edge == 'top' and tgt_edge == 'bottom':
                abutting = abs(src_aabb.y2 - tgt_aabb.y1) < GEOMETRY_TOLERANCE
            elif src_edge == 'bottom' and tgt_edge == 'top':
                abutting = abs(src_aabb.y1 - tgt_aabb.y2) < GEOMETRY_TOLERANCE
            
            if not abutting:
                violations.append(
                    f"H4: D2D PHY not abutting: {conn.source_full} vs {conn.target_full}"
                )
        return violations
    
    def _get_phy_edge_global(self, inst, def_, ip_name) -> Optional[str]:
        """Determine which global edge of the chiplet AABB the PHY is nearest to.
        
        Returns one of: 'left', 'right', 'top', 'bottom'.
        """
        ip_pos = inst.global_ip_position(def_, ip_name)
        if not ip_pos:
            return None
        
        aabb = inst.global_aabb(def_)
        return self._nearest_edge(ip_pos[0], ip_pos[1], aabb)
    
    def _nearest_edge(self, cx: float, cy: float, aabb) -> str:
        """Return the nearest edge name for a point within an AABB."""
        edges = {
            'left': abs(cx - aabb.x1),
            'right': abs(cx - aabb.x2),
            'bottom': abs(cy - aabb.y1),
            'top': abs(cy - aabb.y2),
        }
        return min(edges, key=edges.get)

    def _check_min_spacing(self) -> List[str]:
        """H5: Minimum spacing between adjacent chiplets (seal_ring + scribe_line) on same Z layer.
        
        Does NOT apply to chiplet pairs that have a direct D2D connection (those are handled by H4).
        """
        violations = []
        instances = [inst for inst in self.design.instances if not self._is_base(inst)]
        
        # Build set of chiplet pairs with direct D2D connections.
        # LSI-bridged pairs are included as well: in CoW designs the two top
        # dies of a bridged connection must directly abut (the LSI sits at a
        # lower Z layer and does not interfere), so H5 spacing is waived.
        d2d_pairs = set()
        for conn in self.design.d2d_connections:
            d2d_pairs.add((conn.source_inst, conn.target_inst))
            d2d_pairs.add((conn.target_inst, conn.source_inst))
        
        for i in range(len(instances)):
            def_i = self.design.get_def(instances[i].reference)
            if not def_i:
                continue
            margin = max(def_i.seal_ring) + max(def_i.scribe_line)
            aabb_i = instances[i].global_aabb(def_i).inflate(margin)
            z_i_start = instances[i].pose.z
            z_i_end = z_i_start + def_i.thickness
            
            for j in range(i + 1, len(instances)):
                def_j = self.design.get_def(instances[j].reference)
                if not def_j:
                    continue
                
                # Skip if these two instances have a direct D2D connection
                if (instances[i].name, instances[j].name) in d2d_pairs:
                    continue
                
                # Only check spacing if on same Z layer
                z_j_start = instances[j].pose.z
                z_j_end = z_j_start + def_j.thickness
                if z_i_end <= z_j_start or z_j_end <= z_i_start:
                    continue
                
                margin_j = max(def_j.seal_ring) + max(def_j.scribe_line)
                aabb_j = instances[j].global_aabb(def_j).inflate(margin_j)
                if aabb_i.overlaps(aabb_j, margin=SPACING_OVERLAP_MARGIN):
                    violations.append(
                        f"H5: Insufficient spacing between {instances[i].name} and {instances[j].name}"
                    )
        return violations

    def _check_all_instances_centered(self) -> List[str]:
        """H6: The center of all instances' MBR (except reference) must align with the reference instance's center."""
        violations = []
        
        bboxes = []
        for inst in self.design.instances:
            if self._is_base(inst):
                continue
            def_ = self.design.get_def(inst.reference)
            if def_:
                bboxes.append(inst.global_aabb(def_))
        
        if not bboxes:
            return []
        
        mbr = GeometryEngine.compute_mbr(bboxes)
        mbr_cx = (mbr.x1 + mbr.x2) / 2.0
        mbr_cy = (mbr.y1 + mbr.y2) / 2.0
        
        # Reference center: prefer the actual base instance's AABB center
        # (u_Interposer / uRW), otherwise fall back to the base def size.
        base_inst = self.design.base_instance()
        if base_inst:
            base_def = self.design.get_def(base_inst.reference)
            if base_def:
                ip_cx, ip_cy = base_inst.global_aabb(base_def).center
            else:
                return []
        else:
            base_ref = self._base_ref
            ip_def = self.design.get_def(base_ref) if base_ref else None
            if not ip_def:
                return []
            ip_cx = ip_def.width / 2.0
            ip_cy = ip_def.height / 2.0
        
        if abs(mbr_cx - ip_cx) > GEOMETRY_TOLERANCE or abs(mbr_cy - ip_cy) > GEOMETRY_TOLERANCE:
            violations.append(
                f"H6: Instances MBR center ({mbr_cx:.1f}, {mbr_cy:.1f}) not aligned with "
                f"Interposer center ({ip_cx:.1f}, {ip_cy:.1f})"
            )
        
        return violations

    def _check_lsi_alignment(self) -> List[str]:
        """H7: Bridge LSI center must align with the center of D2D IPs' MBR it bridges.
        
        For each LSI-bridged connection, the LSI instance's MBR center must align with
        the midpoint of the two D2D IP centers.
        """
        violations = []
        
        for conn in self.design.d2d_connections:
            if not conn.lsi_inst:
                continue
            
            lsi_inst = self.design.get_instance(conn.lsi_inst)
            if not lsi_inst:
                continue
            lsi_def = self.design.get_def(lsi_inst.reference)
            if not lsi_def:
                continue
            
            src_inst = self.design.get_instance(conn.source_inst)
            tgt_inst = self.design.get_instance(conn.target_inst)
            if not src_inst or not tgt_inst:
                continue
            
            src_def = self.design.get_def(src_inst.reference)
            tgt_def = self.design.get_def(tgt_inst.reference)
            if not src_def or not tgt_def:
                continue
            
            src_ip_pos = src_inst.global_ip_position(src_def, conn.source_ip)
            tgt_ip_pos = tgt_inst.global_ip_position(tgt_def, conn.target_ip)
            if not src_ip_pos or not tgt_ip_pos:
                continue
            
            lsi_aabb = lsi_inst.global_aabb(lsi_def)
            lsi_cx, lsi_cy = lsi_aabb.center
            
            expected_cx = (src_ip_pos[0] + tgt_ip_pos[0]) / 2.0
            expected_cy = (src_ip_pos[1] + tgt_ip_pos[1]) / 2.0
            
            if abs(lsi_cx - expected_cx) > GEOMETRY_TOLERANCE or abs(lsi_cy - expected_cy) > GEOMETRY_TOLERANCE:
                violations.append(
                    f"H7: LSI {conn.lsi_inst} center ({lsi_cx:.1f}, {lsi_cy:.1f}) not aligned with "
                    f"D2D IP MBR center ({expected_cx:.1f}, {expected_cy:.1f})"
                )
        
        return violations

    # ------------------------------------------------------------------
    # Soft Rules
    # ------------------------------------------------------------------

    def _score_axis_symmetry(self, axis: str) -> Tuple[float, Dict]:
        """Generic axis symmetry scorer. axis='x' for horizontal, 'y' for vertical."""
        is_y = axis == 'y'
        
        instances = [inst for inst in self.design.instances
                       if not self._is_base(inst) and not _is_lsi(inst)]
        
        if not instances:
            formula = f"1 - avg(c{axis}_i - center_{axis}) / (MBR.{'height' if is_y else 'width'}/2)"
            return 1.0, {"formula": formula, "vars": {}, "values": {}, "score": 1.0}
        
        mbr = self.design.mbr_of_instances()
        center = (mbr.y1 + mbr.y2) / 2.0 if is_y else (mbr.x1 + mbr.x2) / 2.0
        max_offset = mbr.height / 2.0 if is_y else mbr.width / 2.0
        
        total_dist = 0.0
        count = 0
        formula = f"1 - avg(c{axis}_i - center_{axis}) / (MBR.{'height' if is_y else 'width'}/2)"
        detail = {"formula": formula, "vars": {}, "values": {}}
        
        for inst in instances:
            chiplet_def = self.design.get_def(inst.reference)
            if not chiplet_def:
                continue
            aabb = inst.global_aabb(chiplet_def)
            coord = aabb.center[1] if is_y else aabb.center[0]
            dist = coord - center
            total_dist += dist
            count += 1
            key = "cy" if is_y else "cx"
            detail["vars"][inst.name] = {key: round(coord, 1), "dist": round(dist, 1)}
        
        if count == 0:
            return 1.0, {"formula": formula, "vars": {}, "values": {}, "score": 1.0}
        
        avg_dist = total_dist / count
        score = 1.0 - (avg_dist / max_offset) if max_offset > 0 else 1.0
        detail["values"] = {
            f"center_{axis}": round(center, 1),
            "max_offset": round(max_offset, 1),
            "avg_dist": round(avg_dist, 1)
        }
        detail["score"] = round(score, 4)
        return score, detail

    def _score_vertical_symmetry(self) -> Tuple[float, Dict]:
        """S1: Vertical symmetry about the horizontal center line (Y-axis symmetry)."""
        return self._score_axis_symmetry('y')

    def _score_horizontal_symmetry(self) -> Tuple[float, Dict]:
        """S2: Horizontal symmetry about the vertical center line (X-axis symmetry)."""
        return self._score_axis_symmetry('x')

    def _score_side_placement(self, keywords: Tuple[str, ...], target_axis: str) -> Tuple[float, Dict]:
        """Generic side placement scorer for HBM/MEM (left/right) or IOD (top/bottom)."""
        is_y = target_axis == 'y'
        
        targets = []
        soc_soics = []
        for inst in self.design.instances:
            if self._is_base(inst):
                continue
            ref_upper = inst.reference.upper()
            if any(kw in ref_upper for kw in keywords):
                targets.append(inst)
            if any(kw in ref_upper for kw in SOC_KEYWORDS):
                soc_soics.append(inst)
        
        if not targets or not soc_soics:
            return 1.0, {"formula": "avg(side_score)", "vars": {}, "values": {}, "score": 1.0}
        
        soc_edges = []
        for soc in soc_soics:
            soc_def = self.design.get_def(soc.reference)
            if soc_def:
                aabb = soc.global_aabb(soc_def)
                soc_edges.append((aabb.y1, aabb.y2) if is_y else (aabb.x1, aabb.x2))
        
        if not soc_edges:
            return 1.0, {"formula": "avg(side_score)", "vars": {}, "values": {}, "score": 1.0}
        
        edge1 = min(e[0] for e in soc_edges)
        edge2 = max(e[1] for e in soc_edges)
        center = (edge1 + edge2) / 2.0
        max_dist = max(abs(edge1 - center), abs(edge2 - center)) * 2
        
        scores = []
        detail = {"formula": "1 - min(|coord - edge1|, |coord - edge2|) / max_dist", "vars": {}, "values": {}}
        
        for target in targets:
            target_def = self.design.get_def(target.reference)
            if not target_def:
                continue
            aabb = target.global_aabb(target_def)
            coord = aabb.center[1] if is_y else aabb.center[0]
            
            dist1 = abs(coord - edge1)
            dist2 = abs(coord - edge2)
            min_dist = min(dist1, dist2)
            
            score = max(0.0, 1.0 - (min_dist / max_dist)) if max_dist > 0 else 1.0
            scores.append(score)
            
            key = "cy" if is_y else "cx"
            detail["vars"][target.name] = {
                key: round(coord, 1),
                "edge1": round(edge1, 1), "edge2": round(edge2, 1),
                "min_dist": round(min_dist, 1), "score": round(score, 4)
            }
        
        final_score = sum(scores) / len(scores) if scores else 1.0
        detail["score"] = round(final_score, 4)
        return final_score, detail

    def _score_hbm_mem_placement(self) -> Tuple[float, Dict]:
        """S3: HBM/MEM should be placed on the left or right side of SOC/SoIC."""
        return self._score_side_placement(HBM_MEM_KEYWORDS, 'x')

    def _score_iod_placement(self) -> Tuple[float, Dict]:
        """S4: IOD should be placed on the top or bottom side of SOC/SoIC."""
        return self._score_side_placement(IOD_KEYWORDS, 'y')

    def _compute_phy_to_edge(self, inst: ChipletInst, ip_name: str) -> Tuple[float, str]:
        """Compute distance from PHY center to the nearest edge of its chiplet.
        
        Returns (distance, edge_name) where edge_name is one of 'left','right','top','bottom'.
        """
        def_ = self.design.get_def(inst.reference)
        if not def_:
            return 0.0, 'right'
        
        # Compute PHY position at origin (0,0) with current orientation
        ip_pos = GeometryEngine.compute_ip_global_position(
            InstancePose(x=0, y=0, z=0, orientation=inst.pose.orientation, flip=inst.pose.flip),
            def_, ip_name)
        if not ip_pos:
            return 0.0, 'right'
        
        # Compute chiplet AABB at origin with current orientation
        aabb = GeometryEngine.compute_global_aabb(
            InstancePose(x=0, y=0, z=0, orientation=inst.pose.orientation, flip=inst.pose.flip),
            def_)
        
        nearest_edge = self._nearest_edge(ip_pos[0], ip_pos[1], aabb)
        edges = {
            'left': abs(ip_pos[0] - aabb.x1),
            'right': abs(ip_pos[0] - aabb.x2),
            'bottom': abs(ip_pos[1] - aabb.y1),
            'top': abs(ip_pos[1] - aabb.y2),
        }
        return edges[nearest_edge], nearest_edge

    def _score_d2d_length_minimize(self) -> Tuple[float, Dict]:
        """S5: Minimize total Manhattan distance of all D2D PHY centers (including LSI-bridged).
        
        Formula: score = 1 - total_excess / max_excess
        where excess = actual_dist - min_dist for each connection
        min_dist = sum of PHY-to-edge distances (when chiplets are abutted)
        """
        if not self.design.d2d_connections:
            return 1.0, {"formula": "1 - total_excess / max_excess", "vars": {}, "values": {}, "score": 1.0}
        
        total_excess = 0.0
        total_dist = 0.0
        count = 0
        conn_details = []
        for conn in self.design.d2d_connections:
            positions = self.design.get_d2d_ip_positions(conn)
            if not positions:
                continue
            (sx, sy), (tx, ty) = positions
            dist = abs(sx - tx) + abs(sy - ty)
            total_dist += dist
            count += 1
            
            # Compute minimum distance (PHY-to-edge distances)
            src_inst = self.design.get_instance(conn.source_inst)
            tgt_inst = self.design.get_instance(conn.target_inst)
            if not src_inst or not tgt_inst:
                continue
            src_to_edge, src_edge = self._compute_phy_to_edge(src_inst, conn.source_ip)
            tgt_to_edge, tgt_edge = self._compute_phy_to_edge(tgt_inst, conn.target_ip)
            min_dist = src_to_edge + tgt_to_edge
            excess = max(0.0, dist - min_dist)
            total_excess += excess
            
            conn_details.append({
                "conn": f"{conn.source_inst}.{conn.source_ip} -> {conn.target_inst}.{conn.target_ip}",
                "sx": round(sx, 1), "sy": round(sy, 1),
                "tx": round(tx, 1), "ty": round(ty, 1),
                "manhattan_dist": round(dist, 1),
                "min_dist": round(min_dist, 1),
                "excess": round(excess, 1),
                "src_edge": src_edge, "tgt_edge": tgt_edge,
            })
        
        if count == 0:
            return 1.0, {"formula": "1 - total_excess / max_excess", "vars": {}, "values": {}, "score": 1.0}
        
        mbr = self.design.mbr_of_instances()
        max_excess = (mbr.width + mbr.height) * count
        score = max(0.0, 1.0 - (total_excess / max_excess)) if max_excess > 0 else 1.0
        
        detail = {
            "formula": "1 - total_excess / ((MBR.width + MBR.height) * count)",
            "vars": {"connections": conn_details},
            "values": {
                "total_dist": round(total_dist, 1),
                "total_excess": round(total_excess, 1),
                "count": count,
                "max_excess": round(max_excess, 1)
            },
            "score": round(score, 4)
        }
        return score, detail
