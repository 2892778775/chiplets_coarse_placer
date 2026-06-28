"""
Placer engine: constraint-driven construction + simulated annealing optimization.

Phase 1: Construct initial legal placement based on chiplet types and constraints.
Phase 2: Refine with SA using only legal geometric moves.
"""

import math
import random
from typing import Dict, List, Tuple, Optional, Set
from .models import DesignModel, ChipletInst, ChipletDef, InstancePose, PlacementSolution, AABB
from .geometry import GeometryEngine
from .constraints import ConstraintChecker


# Constants
DEFAULT_SA_ITERATIONS = 10000
DEFAULT_SA_TEMP = 100.0
DEFAULT_SA_COOLING = 0.995
DEFAULT_ENCLOSURE = 500.0
SPACING_EPSILON = 1.0
INVALID_SCORE = -1e9
MAX_SA_SHIFT = 5000.0
EDGE_NEAR_THRESHOLD = 0.35
EDGE_FAR_THRESHOLD = 0.65
CENTER_ALIGNMENT_TOL = 0.1
INTERPOSER_REF = "Interposer"
SOC_KEYWORDS = ("SOC", "SOIC")
MEM_KEYWORDS = ("MEM", "HBM")
ORIENTATIONS = ["R0", "R90", "R180", "R270"]
FLIPS = ["None", "MX", "MY"]
ORIENT_SCORE_MATCH = 2
ORIENT_SCORE_OPPOSITE = -2
ORIENT_SCORE_ADJACENT = 0.5


class Placer:
    """Main placement engine."""

    def __init__(self, design: DesignModel,
                 algorithm: str = "SA",
                 sa_iterations: int = DEFAULT_SA_ITERATIONS,
                 sa_initial_temp: float = DEFAULT_SA_TEMP,
                 sa_cooling_rate: float = DEFAULT_SA_COOLING,
                 enclosure: float = DEFAULT_ENCLOSURE):
        self.design = design
        self.algorithm = algorithm
        self.sa_iterations = sa_iterations
        self.sa_initial_temp = sa_initial_temp
        self.sa_cooling_rate = sa_cooling_rate
        self.enclosure = enclosure

    def solve(self) -> PlacementSolution:
        """Run full placement based on selected algorithm."""
        if self.algorithm == "Expert":
            expert = ExpertPlacer(self.design, enclosure=self.enclosure)
            return expert.solve()

        self._construct_initial_placement()
        self._simulated_annealing()

        checker = ConstraintChecker(self.design)
        report = checker.check_all()

        mbr = self.design.mbr_of_instances()
        interposer_w = mbr.width + 2 * self.enclosure
        interposer_h = mbr.height + 2 * self.enclosure

        poses = {inst.name: inst.pose.copy() for inst in self.design.instances}

        return PlacementSolution(
            design=self.design,
            instance_poses=poses,
            interposer_size=(interposer_w, interposer_h),
            score=report.total_score,
            report=report
        )

    def _construct_initial_placement(self) -> None:
        """Build a legal initial placement using structured packing."""
        if self.design.d2d_connections:
            expert = ExpertPlacer(self.design, enclosure=self.enclosure)
            expert._analyze_connections()
            expert._determine_orientations()
            expert._place_chiplets()
            expert._align_d2d_connections()
            expert._place_lsi()
            mbr = self.design.mbr_of_instances()
            ip_def = self.design.get_def(INTERPOSER_REF)
            if ip_def:
                ref_cx = ip_def.width / 2.0
                ref_cy = ip_def.height / 2.0
                mbr_cx = (mbr.x1 + mbr.x2) / 2.0
                mbr_cy = (mbr.y1 + mbr.y2) / 2.0
                dx = ref_cx - mbr_cx
                dy = ref_cy - mbr_cy
                for inst in self.design.instances:
                    if inst.reference != INTERPOSER_REF:
                        inst.pose.x += dx
                        inst.pose.y += dy
            return

        self._construct_generic_placement()

    def _get_max_margin(self) -> float:
        """Compute the maximum margin (seal_ring + scribe_line) across all chiplet types."""
        max_margin = 0.0
        for def_ in self.design.chiplet_defs.values():
            if def_.name == INTERPOSER_REF:
                continue
            margin = max(def_.seal_ring) + max(def_.scribe_line)
            if margin > max_margin:
                max_margin = margin
        return max_margin

    def _construct_generic_placement(self) -> None:
        """Generic fallback: pack by Z layer side by side."""
        z_layers = {}
        for inst in self.design.instances:
            if inst.reference == INTERPOSER_REF:
                continue
            z_layers.setdefault(inst.pose.z, []).append(inst)

        sorted_z = sorted(z_layers.keys(), reverse=True)
        base_spacing = self._get_max_margin() * 2 + SPACING_EPSILON

        for z_idx, z in enumerate(sorted_z):
            instances = z_layers[z]
            if not self._check_layer_overlaps(instances):
                continue

            instances.sort(key=lambda inst: inst.pose.x)
            current_x = 0.0
            for inst in instances:
                chiplet_def = self.design.get_def(inst.reference)
                if not chiplet_def:
                    continue
                temp_pose = InstancePose(x=0, y=0, z=0, orientation=inst.pose.orientation, flip=inst.pose.flip)
                aabb = GeometryEngine.compute_global_aabb(temp_pose, chiplet_def)
                inst.pose.x = current_x - aabb.x1
                if z_idx == 0:
                    inst.pose.y = 0.0
                current_x += aabb.width + base_spacing

    def _check_layer_overlaps(self, instances: List[ChipletInst]) -> bool:
        """Check if any two instances in the same layer overlap in XY."""
        for i in range(len(instances)):
            def_i = self.design.get_def(instances[i].reference)
            if not def_i:
                continue
            aabb_i = instances[i].global_aabb(def_i)
            for j in range(i + 1, len(instances)):
                def_j = self.design.get_def(instances[j].reference)
                if not def_j:
                    continue
                if aabb_i.overlaps(instances[j].global_aabb(def_j)):
                    return True
        return False

    def _simulated_annealing(self) -> None:
        """Run SA to optimize soft rules while maintaining hard constraints."""
        flexible_instances = [
            inst for inst in self.design.instances
            if inst.reference != INTERPOSER_REF and inst.flexibility.status != "fixed"
        ]
        if not flexible_instances:
            return

        checker = ConstraintChecker(self.design)
        current_report = checker.check_all()
        current_score = current_report.total_score if current_report.is_valid else INVALID_SCORE

        best_score = current_score
        best_poses = {inst.name: inst.pose.copy() for inst in self.design.instances}
        temp = self.sa_initial_temp

        for _ in range(self.sa_iterations):
            inst = random.choice(flexible_instances)
            chiplet_def = self.design.get_def(inst.reference)
            if not chiplet_def:
                continue

            old_pose = inst.pose.copy()
            move_type = random.choice(["shift", "rotate", "swap"])

            if move_type == "shift" and inst.flexibility.shift:
                self._try_shift(inst)
            elif move_type == "rotate" and inst.flexibility.rotate:
                self._try_rotate(inst)
            elif move_type == "swap" and len(flexible_instances) > 1:
                other = random.choice([f for f in flexible_instances if f != inst])
                self._try_swap(inst, other)

            new_report = checker.check_all()
            if not new_report.is_valid:
                inst.pose = old_pose
                continue

            new_score = new_report.total_score
            delta = new_score - current_score

            if delta > 0 or random.random() < math.exp(delta / temp):
                current_score = new_score
                if current_score > best_score:
                    best_score = current_score
                    best_poses = {i.name: i.pose.copy() for i in self.design.instances}
            else:
                inst.pose = old_pose

            temp *= self.sa_cooling_rate

        for inst in self.design.instances:
            if inst.name in best_poses:
                inst.pose = best_poses[inst.name].copy()

    def _try_shift(self, inst: ChipletInst) -> None:
        """Try to shift an instance by a small random amount."""
        inst.pose.x += random.uniform(-MAX_SA_SHIFT, MAX_SA_SHIFT)
        inst.pose.y += random.uniform(-MAX_SA_SHIFT, MAX_SA_SHIFT)

    def _try_rotate(self, inst: ChipletInst) -> None:
        """Try to rotate an instance by 90 degrees."""
        current_idx = ORIENTATIONS.index(inst.pose.orientation) if inst.pose.orientation in ORIENTATIONS else 0
        inst.pose.orientation = ORIENTATIONS[(current_idx + random.choice([1, -1])) % 4]

    def _try_swap(self, inst_a: ChipletInst, inst_b: ChipletInst) -> None:
        """Swap positions of two instances."""
        inst_a.pose.x, inst_b.pose.x = inst_b.pose.x, inst_a.pose.x
        inst_a.pose.y, inst_b.pose.y = inst_b.pose.y, inst_a.pose.y


class BasePlacer:
    """Base placer with common geometric utilities and helpers."""

    def __init__(self, design: DesignModel, enclosure: float = DEFAULT_ENCLOSURE):
        self.design = design
        self.enclosure = enclosure
        self.ip_cache = {}

    def _get_margins(self, def_: ChipletDef) -> Dict[str, float]:
        return {
            'l': def_.seal_ring[0] + def_.scribe_line[0] + SPACING_EPSILON,
            'r': def_.seal_ring[1] + def_.scribe_line[1] + SPACING_EPSILON,
            't': def_.seal_ring[2] + def_.scribe_line[2] + SPACING_EPSILON,
            'b': def_.seal_ring[3] + def_.scribe_line[3] + SPACING_EPSILON,
        }

    def _get_ip_info(self, inst_name: str, ip_name: str) -> Optional[Dict]:
        """Return IP local info: loc, size, chiplet dimensions."""
        key = (inst_name, ip_name)
        if key in self.ip_cache:
            return self.ip_cache[key]

        inst = self.design.get_instance(inst_name)
        if not inst:
            return None
        def_ = self.design.get_def(inst.reference)
        if not def_:
            return None

        entry = None
        for e in def_.omap_entries:
            if e.name == ip_name:
                entry = e
                break

        if not entry:
            import re
            m = re.search(r'(\d+)$', ip_name)
            if m:
                idx = int(m.group(1))
                if idx < len(def_.omap_entries):
                    entry = def_.omap_entries[idx]

        if not entry and def_.omap_entries:
            entry = def_.omap_entries[0]

        if not entry:
            return None

        obj_size = def_.get_object_size(entry.obj_type)
        info = {
            'loc_x': entry.loc_x,
            'loc_y': entry.loc_y,
            'ip_w': obj_size[0],
            'ip_h': obj_size[1],
            'ip_cx': entry.loc_x + obj_size[0] / 2.0,
            'ip_cy': entry.loc_y + obj_size[1] / 2.0,
            'chiplet_w': def_.width,
            'chiplet_h': def_.height,
        }
        self.ip_cache[key] = info
        return info

    def _get_ip_edge(self, ip_cx: float, ip_cy: float, cw: float, ch: float) -> List[str]:
        """Determine which edge(s) the IP sits on."""
        edges = []
        if ip_cx < cw * EDGE_NEAR_THRESHOLD:
            edges.append('left')
        elif ip_cx > cw * EDGE_FAR_THRESHOLD:
            edges.append('right')
        if ip_cy < ch * EDGE_NEAR_THRESHOLD:
            edges.append('bottom')
        elif ip_cy > ch * EDGE_FAR_THRESHOLD:
            edges.append('top')
        return edges

    def _get_ip_edge_transformed(self, inst_name: str, ip_name: str, orient: str, flip: str) -> List[str]:
        """Get IP edge after applying orientation + flip."""
        info = self._get_ip_info(inst_name, ip_name)
        if not info:
            return []

        pose = InstancePose(x=0, y=0, z=0, orientation=orient, flip=flip)
        gx, gy = GeometryEngine.local_to_global(pose, info['ip_cx'], info['ip_cy'],
                                                info['chiplet_w'], info['chiplet_h'])

        corners = [(0, 0), (info['chiplet_w'], 0), (0, info['chiplet_h']), (info['chiplet_w'], info['chiplet_h'])]
        transformed = [GeometryEngine.local_to_global(pose, cx, cy, info['chiplet_w'], info['chiplet_h']) for cx, cy in corners]
        min_x = min(c[0] for c in transformed)
        max_x = max(c[0] for c in transformed)
        min_y = min(c[1] for c in transformed)
        max_y = max(c[1] for c in transformed)
        tw = max_x - min_x
        th = max_y - min_y

        return self._get_ip_edge(gx - min_x, gy - min_y, tw, th)

    @staticmethod
    def _opposite(direction: str) -> str:
        return {'right': 'left', 'left': 'right', 'top': 'bottom', 'bottom': 'top'}[direction]

    def _get_phy_edge_global(self, inst, def_, ip_name) -> Optional[str]:
        """Determine which global edge of the chiplet AABB the PHY is nearest to."""
        ip_pos = inst.global_ip_position(def_, ip_name)
        if not ip_pos:
            return None

        aabb = inst.global_aabb(def_)
        edges = {
            'left': abs(ip_pos[0] - aabb.x1),
            'right': abs(ip_pos[0] - aabb.x2),
            'bottom': abs(ip_pos[1] - aabb.y1),
            'top': abs(ip_pos[1] - aabb.y2),
        }
        return min(edges, key=edges.get)

    def _choose_best_orientation(self, inst_name: str, desired: Dict[str, str]) -> Tuple[str, str]:
        """Brute-force search for best orientation/flip combination."""
        best_score = -1
        best_orient, best_flip = 'R0', 'None'

        for orient in ORIENTATIONS:
            for flip in FLIPS:
                score = 0
                for ip_name, direction in desired.items():
                    edges = self._get_ip_edge_transformed(inst_name, ip_name, orient, flip)
                    if not edges:
                        continue
                    if direction in edges:
                        score += ORIENT_SCORE_MATCH
                    elif self._opposite(direction) in edges:
                        score += ORIENT_SCORE_OPPOSITE
                    else:
                        score += ORIENT_SCORE_ADJACENT

                if score > best_score:
                    best_score = score
                    best_orient = orient
                    best_flip = flip

        return best_orient, best_flip


class ExpertPlacer(BasePlacer):
    """
    Expert-based placement following the 8-step expert rules algorithm:
    1. Dominant instances recognition
    2. Design partition
    3. Isolate instances plan
    4. Dominant instance placement
    5. Slave instances placement
    6. Isolate instances placement
    7. Design merge
    8. Merged design placement
    """

    class Group:
        """A placement group containing a dominant instance and its slaves."""
        def __init__(self, idx: int, aabb: AABB):
            self.idx = idx
            self.aabb = aabb
            self.dominant_name: Optional[str] = None
            self.slave_names: List[str] = []
            self.isolated_names: List[str] = []
            self.dominant_edge: Optional[str] = None

    def __init__(self, design: DesignModel, enclosure: float = DEFAULT_ENCLOSURE):
        super().__init__(design, enclosure)
        self.dominant_names: Set[str] = set()
        self.slave_map: Dict[str, List[str]] = {}
        self.isolated_names: Set[str] = set()
        self.lsi_names: Set[str] = set()
        self.groups: List[ExpertPlacer.Group] = []
        self.instance_to_group: Dict[str, int] = {}
        self._phase = 0

    def solve(self) -> PlacementSolution:
        """Run the full 8-step expert placement pipeline."""
        self._analyze_connections()
        self._determine_orientations()
        self._place_chiplets()
        self._align_d2d_connections()
        self._place_lsi()
        self._center_all_instances()

        checker = ConstraintChecker(self.design)
        report = checker.check_all()

        mbr = self.design.mbr_of_instances()
        interposer_w = mbr.width + 2 * self.enclosure
        interposer_h = mbr.height + 2 * self.enclosure

        poses = {inst.name: inst.pose.copy() for inst in self.design.instances}

        return PlacementSolution(
            design=self.design,
            instance_poses=poses,
            interposer_size=(interposer_w, interposer_h),
            score=report.total_score,
            report=report
        )

    def _analyze_connections(self) -> None:
        """Legacy wrapper: runs Steps 1-3."""
        self._step1_dominant_instances()
        self._step2_design_partition()
        self._step3_isolate_instances_plan()
        self._phase = 3

    def _determine_orientations(self) -> None:
        """Legacy wrapper: runs Step 4 (dominant placement includes orientation)."""
        if self._phase < 3:
            self._analyze_connections()
        self._step4_dominant_placement()
        self._phase = 4

    def _place_chiplets(self) -> None:
        """Legacy wrapper: runs Step 5 (slave placement)."""
        if self._phase < 4:
            self._determine_orientations()
        self._step5_slave_placement()
        self._phase = 5

    def _align_d2d_connections(self) -> None:
        """Legacy wrapper: no-op; alignment is handled during Step 5."""
        if self._phase < 5:
            self._place_chiplets()
        self._phase = 6

    def _place_lsi(self) -> None:
        """Legacy wrapper: places LSI and runs Steps 6-7."""
        if self._phase < 6:
            self._align_d2d_connections()
        self._place_lsi_internal()
        self._step6_isolate_placement()
        self._step7_design_merge()
        self._resolve_overlaps_and_spacing()
        self._place_lsi_internal()
        self._phase = 7

    def _center_all_instances(self) -> None:
        """Legacy wrapper: runs Step 8."""
        if self._phase < 7:
            self._place_lsi()
        self._step8_center_design()
        self._phase = 8

    # ------------------------------------------------------------------
    # Step 1: Dominant instances recognition
    # ------------------------------------------------------------------

    def _step1_dominant_instances(self) -> None:
        """Identify dominant instances, slaves, and isolated instances."""
        degrees: Dict[str, int] = {}
        for inst in self.design.instances:
            if inst.reference == INTERPOSER_REF:
                continue
            degrees[inst.name] = 0

        for conn in self.design.d2d_connections:
            if conn.source_inst in degrees:
                degrees[conn.source_inst] += 1
            if conn.target_inst in degrees:
                degrees[conn.target_inst] += 1

        max_degree = max(degrees.values()) if degrees else 0

        has_soc = any(
            any(kw in self.design.get_instance(name).reference.upper() for kw in SOC_KEYWORDS)
            for name in degrees if self.design.get_instance(name)
        )

        self.dominant_names = set()
        for name, deg in degrees.items():
            inst = self.design.get_instance(name)
            if not inst:
                continue
            ref_upper = inst.reference.upper()
            if any(kw in ref_upper for kw in SOC_KEYWORDS):
                self.dominant_names.add(name)
            elif not has_soc and deg == max_degree and max_degree > 0:
                self.dominant_names.add(name)

        if not self.dominant_names:
            for name, deg in degrees.items():
                if deg > 0:
                    self.dominant_names.add(name)

        self.slave_map = {name: [] for name in self.dominant_names}
        for name, deg in degrees.items():
            if name in self.dominant_names or deg == 0:
                continue

            conn_counts: Dict[str, int] = {}
            for conn in self.design.d2d_connections:
                if conn.source_inst == name and conn.target_inst in self.dominant_names:
                    conn_counts[conn.target_inst] = conn_counts.get(conn.target_inst, 0) + 1
                elif conn.target_inst == name and conn.source_inst in self.dominant_names:
                    conn_counts[conn.source_inst] = conn_counts.get(conn.source_inst, 0) + 1

            if conn_counts:
                best = max(conn_counts.items(), key=lambda x: (x[1], degrees.get(x[0], 0)))
                self.slave_map[best[0]].append(name)
            else:
                inst = self.design.get_instance(name)
                best_dominant = None
                best_dist = float('inf')
                for dom_name in self.dominant_names:
                    dom_inst = self.design.get_instance(dom_name)
                    if dom_inst and inst:
                        dist = math.hypot(inst.pose.x - dom_inst.pose.x, inst.pose.y - dom_inst.pose.y)
                        if dist < best_dist:
                            best_dist = dist
                            best_dominant = dom_name
                if best_dominant:
                    self.slave_map[best_dominant].append(name)

        self.isolated_names = set()
        self.lsi_names = set()
        for name, deg in degrees.items():
            if name in self.dominant_names:
                continue
            if any(name in slaves for slaves in self.slave_map.values()):
                continue
            if deg == 0:
                inst = self.design.get_instance(name)
                if inst and "LSI" in inst.reference.upper():
                    self.lsi_names.add(name)
                else:
                    self.isolated_names.add(name)

    # ------------------------------------------------------------------
    # Step 2: Design partition
    # ------------------------------------------------------------------

    def _step2_design_partition(self) -> None:
        """Partition the Interposer into N groups for N dominant instances."""
        ip_def = self.design.get_def(INTERPOSER_REF)
        ip_w = ip_def.width if ip_def else 64000
        ip_h = ip_def.height if ip_def else 35000

        N = len(self.dominant_names)
        self.groups = []

        if N <= 1:
            self.groups.append(self.Group(0, AABB(0, 0, ip_w, ip_h)))
        elif N == 2:
            if ip_w >= ip_h:
                self.groups.append(self.Group(0, AABB(0, 0, ip_w / 2.0, ip_h)))
                self.groups.append(self.Group(1, AABB(ip_w / 2.0, 0, ip_w, ip_h)))
            else:
                self.groups.append(self.Group(0, AABB(0, 0, ip_w, ip_h / 2.0)))
                self.groups.append(self.Group(1, AABB(0, ip_h / 2.0, ip_w, ip_h)))
        else:
            best_diff = float('inf')
            best_rows, best_cols = 1, N
            for rows in range(1, int(math.sqrt(N)) + 1):
                if N % rows == 0:
                    cols = N // rows
                    diff = abs(rows - cols)
                    if diff < best_diff:
                        best_diff = diff
                        best_rows, best_cols = rows, cols

            cell_w = ip_w / best_cols
            cell_h = ip_h / best_rows
            idx = 0
            for r in range(best_rows):
                for c in range(best_cols):
                    aabb = AABB(c * cell_w, r * cell_h, (c + 1) * cell_w, (r + 1) * cell_h)
                    self.groups.append(self.Group(idx, aabb))
                    idx += 1

        sorted_dominants = sorted(self.dominant_names)
        for i, name in enumerate(sorted_dominants):
            if i < len(self.groups):
                self.groups[i].dominant_name = name
                self.instance_to_group[name] = i

        for dom_name, slaves in self.slave_map.items():
            g_idx = self.instance_to_group.get(dom_name)
            if g_idx is not None:
                for slave_name in slaves:
                    self.groups[g_idx].slave_names.append(slave_name)
                    self.instance_to_group[slave_name] = g_idx

        for lsi_name in self.lsi_names:
            for conn in self.design.d2d_connections:
                if conn.lsi_inst == lsi_name:
                    src_group = self.instance_to_group.get(conn.source_inst)
                    if src_group is not None:
                        self.groups[src_group].isolated_names.append(lsi_name)
                        self.instance_to_group[lsi_name] = src_group
                        break
                    tgt_group = self.instance_to_group.get(conn.target_inst)
                    if tgt_group is not None:
                        self.groups[tgt_group].isolated_names.append(lsi_name)
                        self.instance_to_group[lsi_name] = tgt_group
                        break

    # ------------------------------------------------------------------
    # Step 3: Isolate instances plan
    # ------------------------------------------------------------------

    def _step3_isolate_instances_plan(self) -> None:
        """Distribute isolated instances (IOD, IPD, DUMMY) evenly across groups."""
        isolated_by_type: Dict[str, List[str]] = {}
        for name in self.isolated_names:
            inst = self.design.get_instance(name)
            ref = inst.reference
            if ref not in isolated_by_type:
                isolated_by_type[ref] = []
            isolated_by_type[ref].append(name)

        N = len(self.groups)
        for ref, names in isolated_by_type.items():
            count = len(names)
            per_group = count // N
            remainder = count % N

            name_idx = 0
            for g_idx, g in enumerate(self.groups):
                n_assign = per_group + (1 if g_idx < remainder else 0)
                for _ in range(n_assign):
                    if name_idx < len(names):
                        g.isolated_names.append(names[name_idx])
                        self.instance_to_group[names[name_idx]] = g.idx
                        name_idx += 1

    # ------------------------------------------------------------------
    # Step 4: Dominant instance placement
    # ------------------------------------------------------------------

    def _step4_dominant_placement(self) -> None:
        """Place each dominant instance in its group with proper orientation and position."""
        for group in self.groups:
            if not group.dominant_name:
                continue

            inst = self.design.get_instance(group.dominant_name)
            def_ = self.design.get_def(inst.reference)
            if not def_:
                continue

            analysis = self._analyze_dominant_ips(group.dominant_name)

            place_at_center = False
            if analysis:
                same_type = analysis.get('same_type', False)
                balanced = analysis.get('balanced', False)
                place_at_center = same_type and balanced

            has_cross_group = False
            target_edge = None
            for conn in self.design.d2d_connections:
                if conn.source_inst == group.dominant_name:
                    partner = conn.target_inst
                elif conn.target_inst == group.dominant_name:
                    partner = conn.source_inst
                else:
                    continue

                partner_g_idx = self.instance_to_group.get(partner)
                if partner_g_idx is not None and partner_g_idx != group.idx:
                    has_cross_group = True
                    partner_group = self.groups[partner_g_idx]
                    group_cx = (group.aabb.x1 + group.aabb.x2) / 2.0
                    partner_cx = (partner_group.aabb.x1 + partner_group.aabb.x2) / 2.0
                    if partner_cx > group_cx:
                        target_edge = 'right'
                    else:
                        target_edge = 'left'

            group.dominant_edge = None
            if place_at_center or not analysis:
                group.dominant_edge = 'center'
            elif has_cross_group and target_edge:
                group.dominant_edge = target_edge
            else:
                edges = analysis['edges']
                max_edge = max(edges, key=edges.get) if edges else 'right'
                group.dominant_edge = max_edge

            orient, flip = self._choose_orientation_for_dominant(group.dominant_name, analysis)
            inst.pose.orientation = orient
            inst.pose.flip = flip

            aabb = inst.global_aabb(def_)

            if group.dominant_edge == 'center':
                cx = (group.aabb.x1 + group.aabb.x2) / 2.0
                cy = (group.aabb.y1 + group.aabb.y2) / 2.0
                inst.pose.x = cx - aabb.width / 2.0
                inst.pose.y = cy - aabb.height / 2.0
            elif group.dominant_edge == 'right':
                inst.pose.x = group.aabb.x2 - aabb.width
                inst.pose.y = (group.aabb.y1 + group.aabb.y2) / 2.0 - aabb.height / 2.0
            elif group.dominant_edge == 'left':
                inst.pose.x = group.aabb.x1
                inst.pose.y = (group.aabb.y1 + group.aabb.y2) / 2.0 - aabb.height / 2.0
            elif group.dominant_edge == 'top':
                inst.pose.x = (group.aabb.x1 + group.aabb.x2) / 2.0 - aabb.width / 2.0
                inst.pose.y = group.aabb.y2 - aabb.height
            elif group.dominant_edge == 'bottom':
                inst.pose.x = (group.aabb.x1 + group.aabb.x2) / 2.0 - aabb.width / 2.0
                inst.pose.y = group.aabb.y1
            else:
                cx = (group.aabb.x1 + group.aabb.x2) / 2.0
                cy = (group.aabb.y1 + group.aabb.y2) / 2.0
                inst.pose.x = cx - aabb.width / 2.0
                inst.pose.y = cy - aabb.height / 2.0

    # ------------------------------------------------------------------
    # Step 4 helpers
    # ------------------------------------------------------------------

    def _analyze_dominant_ips(self, inst_name: str) -> Optional[Dict]:
        """Analyze D2D IP distribution on a dominant instance."""
        inst = self.design.get_instance(inst_name)
        def_ = self.design.get_def(inst.reference)
        if not def_:
            return None

        edges = {'left': 0, 'right': 0, 'top': 0, 'bottom': 0}
        edge_y = {'left': [], 'right': []}
        edge_x = {'top': [], 'bottom': []}
        partner_types = set()
        last_info = None

        for conn in self.design.d2d_connections:
            if conn.source_inst == inst_name:
                ip_name = conn.source_ip
                partner = self.design.get_instance(conn.target_inst)
            elif conn.target_inst == inst_name:
                ip_name = conn.target_ip
                partner = self.design.get_instance(conn.source_inst)
            else:
                continue

            info = self._get_ip_info(inst_name, ip_name)
            if not info:
                continue
            last_info = info

            ip_edges = self._get_ip_edge(info['ip_cx'], info['ip_cy'],
                                          info['chiplet_w'], info['chiplet_h'])
            for e in ip_edges:
                edges[e] += 1
                if e in ('left', 'right'):
                    edge_y[e].append(info['ip_cy'])
                if e in ('top', 'bottom'):
                    edge_x[e].append(info['ip_cx'])

            if partner:
                partner_types.add(partner.reference)

        balanced = False
        ch = last_info['chiplet_h'] if last_info else def_.height
        cw = last_info['chiplet_w'] if last_info else def_.width
        if edges['left'] == edges['right'] and edges['left'] > 0:
            left_center = sum(edge_y['left']) / len(edge_y['left']) if edge_y['left'] else ch / 2
            right_center = sum(edge_y['right']) / len(edge_y['right']) if edge_y['right'] else ch / 2
            if abs(left_center - ch / 2) < ch * CENTER_ALIGNMENT_TOL and abs(right_center - ch / 2) < ch * CENTER_ALIGNMENT_TOL:
                balanced = True
        if edges['top'] == edges['bottom'] and edges['top'] > 0:
            top_center = sum(edge_x['top']) / len(edge_x['top']) if edge_x['top'] else cw / 2
            bottom_center = sum(edge_x['bottom']) / len(edge_x['bottom']) if edge_x['bottom'] else cw / 2
            if abs(top_center - cw / 2) < cw * CENTER_ALIGNMENT_TOL and abs(bottom_center - cw / 2) < cw * CENTER_ALIGNMENT_TOL:
                balanced = True

        same_type = len(partner_types) <= 1
        has_mem_hbm = any(any(kw in pt.upper() for kw in MEM_KEYWORDS) for pt in partner_types)

        return {
            'edges': edges,
            'balanced': balanced,
            'same_type': same_type,
            'has_mem_hbm': has_mem_hbm,
            'partner_types': partner_types
        }

    def _choose_orientation_for_dominant(self, inst_name: str, analysis: Optional[Dict]) -> Tuple[str, str]:
        """Choose orientation for dominant instance based on D2D connections and group assignment."""
        inst = self.design.get_instance(inst_name)
        def_ = self.design.get_def(inst.reference)
        if not def_:
            return 'R0', 'None'

        if not analysis or not analysis.get('partner_types'):
            return 'R0', 'None'

        desired = {}
        g_idx = self.instance_to_group.get(inst_name)
        group = self.groups[g_idx] if g_idx is not None and g_idx < len(self.groups) else None

        for conn in self.design.d2d_connections:
            if conn.source_inst == inst_name:
                ip_name = conn.source_ip
                partner_name = conn.target_inst
            elif conn.target_inst == inst_name:
                ip_name = conn.target_ip
                partner_name = conn.source_inst
            else:
                continue

            partner = self.design.get_instance(partner_name)
            if not partner:
                continue

            partner_g_idx = self.instance_to_group.get(partner_name)

            if partner_g_idx is None or g_idx is None:
                partner_ref = partner.reference.upper()
                if any(kw in partner_ref for kw in MEM_KEYWORDS):
                    desired[ip_name] = 'right'
                else:
                    ip_info = self._get_ip_info(inst_name, ip_name)
                    if ip_info:
                        ip_edges = self._get_ip_edge(ip_info['ip_cx'], ip_info['ip_cy'],
                                                      ip_info['chiplet_w'], ip_info['chiplet_h'])
                        desired[ip_name] = ip_edges[0] if ip_edges else 'right'
                    else:
                        desired[ip_name] = 'right'
            elif partner_g_idx == g_idx:
                if conn.has_lsi and group and group.dominant_edge:
                    if group.dominant_edge == 'right':
                        desired[ip_name] = 'left'
                    elif group.dominant_edge == 'left':
                        desired[ip_name] = 'right'
                    else:
                        ip_info = self._get_ip_info(inst_name, ip_name)
                        if ip_info:
                            if ip_info['ip_cx'] < ip_info['chiplet_w'] * EDGE_NEAR_THRESHOLD:
                                desired[ip_name] = 'left'
                            elif ip_info['ip_cx'] > ip_info['chiplet_w'] * EDGE_FAR_THRESHOLD:
                                desired[ip_name] = 'right'
                            else:
                                desired[ip_name] = 'right'
                        else:
                            desired[ip_name] = 'right'
                else:
                    ip_info = self._get_ip_info(inst_name, ip_name)
                    if ip_info:
                        if ip_info['ip_cx'] < ip_info['chiplet_w'] * EDGE_NEAR_THRESHOLD:
                            desired[ip_name] = 'left'
                        elif ip_info['ip_cx'] > ip_info['chiplet_w'] * EDGE_FAR_THRESHOLD:
                            desired[ip_name] = 'right'
                        elif ip_info['ip_cy'] < ip_info['chiplet_h'] * EDGE_NEAR_THRESHOLD:
                            desired[ip_name] = 'bottom'
                        elif ip_info['ip_cy'] > ip_info['chiplet_h'] * EDGE_FAR_THRESHOLD:
                            desired[ip_name] = 'top'
                        else:
                            desired[ip_name] = 'right'
                    else:
                        desired[ip_name] = 'right'
            else:
                partner_group = self.groups[partner_g_idx]
                group = self.groups[g_idx]
                group_cx = (group.aabb.x1 + group.aabb.x2) / 2.0
                partner_cx = (partner_group.aabb.x1 + partner_group.aabb.x2) / 2.0
                if partner_cx > group_cx:
                    desired[ip_name] = 'right'
                else:
                    desired[ip_name] = 'left'

        if desired:
            return self._choose_best_orientation(inst_name, desired)
        return 'R0', 'None'

    # ------------------------------------------------------------------
    # Step 5: Slave instances placement
    # ------------------------------------------------------------------

    def _step5_slave_placement(self) -> None:
        """Place slave instances to abut and align with their dominant instance."""
        for group in self.groups:
            if not group.dominant_name:
                continue

            dominant = self.design.get_instance(group.dominant_name)
            dominant_def = self.design.get_def(dominant.reference)
            if not dominant_def:
                continue

            for slave_name in group.slave_names:
                slave = self.design.get_instance(slave_name)
                slave_def = self.design.get_def(slave.reference)
                if not slave_def:
                    continue

                conns = []
                for conn in self.design.d2d_connections:
                    if (conn.source_inst == group.dominant_name and conn.target_inst == slave_name) or \
                       (conn.source_inst == slave_name and conn.target_inst == group.dominant_name):
                        conns.append(conn)

                if not conns:
                    self._place_slave_near_dominant(dominant, dominant_def, slave, slave_def)
                    continue

                desired = {}
                for conn in conns:
                    slv_ip = conn.source_ip if conn.source_inst == slave_name else conn.target_ip
                    if conn.has_lsi and group.dominant_edge:
                        if group.dominant_edge == 'right':
                            desired[slv_ip] = 'right'
                        elif group.dominant_edge == 'left':
                            desired[slv_ip] = 'left'
                        else:
                            side = self._determine_slave_side(slave_name, group.dominant_name)
                            if side:
                                desired[slv_ip] = side
                    else:
                        side = self._determine_slave_side(slave_name, group.dominant_name)
                        if side:
                            desired[slv_ip] = side

                if desired:
                    orient, flip = self._choose_best_orientation(slave.name, desired)
                    slave.pose.orientation = orient
                    slave.pose.flip = flip

                self._place_slave_for_connection(dominant, dominant_def, slave, slave_def, conns[0])

    def _determine_slave_side(self, slave_name: str, dominant_name: str) -> Optional[str]:
        """Determine which side of the dominant the slave should be placed on."""
        for conn in self.design.d2d_connections:
            if (conn.source_inst == dominant_name and conn.target_inst == slave_name) or \
               (conn.source_inst == slave_name and conn.target_inst == dominant_name):
                dom_ip = conn.source_ip if conn.source_inst == dominant_name else conn.target_ip
                dom_inst = self.design.get_instance(dominant_name)
                dom_def = self.design.get_def(dom_inst.reference)
                if dom_inst and dom_def:
                    dom_edge = self._get_phy_edge_global(dom_inst, dom_def, dom_ip)
                    if dom_edge:
                        return self._opposite(dom_edge)
        return 'right'

    def _place_slave_for_connection(self, dominant, dominant_def, slave, slave_def, conn):
        """Place slave instance so it abuts and aligns with the dominant for a given connection."""
        dom_ip = conn.source_ip if conn.source_inst == dominant.name else conn.target_ip
        slv_ip = conn.target_ip if conn.target_inst == slave.name else conn.source_ip

        dom_ip_pos = dominant.global_ip_position(dominant_def, dom_ip)
        if not dom_ip_pos:
            return

        pose = InstancePose(x=0, y=0, z=0, orientation=slave.pose.orientation, flip=slave.pose.flip)
        slv_info = self._get_ip_info(slave.name, slv_ip)
        if not slv_info:
            return

        local_cx = slv_info['loc_x'] + slv_info['ip_w'] / 2.0
        local_cy = slv_info['loc_y'] + slv_info['ip_h'] / 2.0
        slave_ip_at_origin = GeometryEngine.local_to_global(pose, local_cx, local_cy,
                                                             slave_def.width, slave_def.height)
        slave_aabb_origin = GeometryEngine.compute_global_aabb(pose, slave_def)

        dom_edge = self._get_phy_edge_global(dominant, dominant_def, dom_ip)
        slv_edges = self._get_ip_edge_transformed(slave.name, slv_ip, slave.pose.orientation, slave.pose.flip)
        slv_edge = slv_edges[0] if slv_edges else 'left'

        dom_aabb = dominant.global_aabb(dominant_def)

        if dom_edge == 'right' and slv_edge == 'left':
            slave.pose.x = dom_aabb.x2
            slave.pose.y = dom_ip_pos[1] - slave_ip_at_origin[1]
        elif dom_edge == 'left' and slv_edge == 'right':
            slave.pose.x = dom_aabb.x1 - slave_aabb_origin.width
            slave.pose.y = dom_ip_pos[1] - slave_ip_at_origin[1]
        elif dom_edge == 'top' and slv_edge == 'bottom':
            slave.pose.x = dom_ip_pos[0] - slave_ip_at_origin[0]
            slave.pose.y = dom_aabb.y2
        elif dom_edge == 'bottom' and slv_edge == 'top':
            slave.pose.x = dom_ip_pos[0] - slave_ip_at_origin[0]
            slave.pose.y = dom_aabb.y1 - slave_aabb_origin.height
        else:
            slave.pose.x = dom_aabb.x2
            slave.pose.y = dom_ip_pos[1] - slave_ip_at_origin[1]

    def _place_slave_near_dominant(self, dominant, dominant_def, slave, slave_def):
        """Place a slave near its dominant with minimum spacing when no direct D2D connection."""
        margins = self._get_margins(dominant_def)
        dom_aabb = dominant.global_aabb(dominant_def)
        slave_aabb = slave.global_aabb(slave_def)

        slave.pose.x = dom_aabb.x2 + margins['r']
        slave.pose.y = dom_aabb.center[1] - slave_aabb.height / 2.0

    # ------------------------------------------------------------------
    # LSI placement (bridged connections)
    # ------------------------------------------------------------------

    def _place_lsi_internal(self) -> None:
        """Place each LSI centered under its D2D IP group."""
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

            center_x = (src_ip_pos[0] + tgt_ip_pos[0]) / 2.0
            center_y = (src_ip_pos[1] + tgt_ip_pos[1]) / 2.0

            lsi_inst.pose.x = center_x - lsi_def.width / 2.0
            lsi_inst.pose.y = center_y - lsi_def.height / 2.0
            lsi_inst.pose.orientation = 'R0'
            lsi_inst.pose.flip = 'None'

    # ------------------------------------------------------------------
    # Step 6: Isolate instances placement
    # ------------------------------------------------------------------

    def _step6_isolate_placement(self) -> None:
        """Place isolated instances (IOD, IPD, DUMMY) in available gaps."""
        for group in self.groups:
            if not group.isolated_names:
                continue

            func_names = []
            if group.dominant_name:
                func_names.append(group.dominant_name)
            func_names.extend(group.slave_names)
            func_names.extend([n for n in group.isolated_names if n in self.lsi_names])

            func_aabbs = []
            for name in func_names:
                inst = self.design.get_instance(name)
                def_ = self.design.get_def(inst.reference)
                if def_:
                    func_aabbs.append(inst.global_aabb(def_))

            if not func_aabbs:
                self._place_isolated_in_empty_group(group)
                continue

            func_mbr = GeometryEngine.compute_mbr(func_aabbs)

            gaps = []
            if func_mbr.x1 > group.aabb.x1:
                gaps.append(AABB(group.aabb.x1, group.aabb.y1, func_mbr.x1, group.aabb.y2))
            if func_mbr.x2 < group.aabb.x2:
                gaps.append(AABB(func_mbr.x2, group.aabb.y1, group.aabb.x2, group.aabb.y2))
            if func_mbr.y1 > group.aabb.y1:
                gaps.append(AABB(group.aabb.x1, group.aabb.y1, group.aabb.x2, func_mbr.y1))
            if func_mbr.y2 < group.aabb.y2:
                gaps.append(AABB(group.aabb.x1, func_mbr.y2, group.aabb.x2, group.aabb.y2))

            for i in range(len(func_aabbs)):
                for j in range(i + 1, len(func_aabbs)):
                    a = func_aabbs[i]
                    b = func_aabbs[j]
                    if a.y2 > b.y1 and b.y2 > a.y1:
                        if b.x1 > a.x2:
                            gap = AABB(a.x2, max(a.y1, b.y1), b.x1, min(a.y2, b.y2))
                            if gap.width > 0 and gap.height > 0:
                                gaps.append(gap)
                        elif a.x1 > b.x2:
                            gap = AABB(b.x2, max(a.y1, b.y1), a.x1, min(a.y2, b.y2))
                            if gap.width > 0 and gap.height > 0:
                                gaps.append(gap)
                    if a.x2 > b.x1 and b.x2 > a.x1:
                        if b.y1 > a.y2:
                            gap = AABB(max(a.x1, b.x1), a.y2, min(a.x2, b.x2), b.y1)
                            if gap.width > 0 and gap.height > 0:
                                gaps.append(gap)
                        elif a.y1 > b.y2:
                            gap = AABB(max(a.x1, b.x1), b.y2, min(a.x2, b.x2), a.y1)
                            if gap.width > 0 and gap.height > 0:
                                gaps.append(gap)

            gaps = [g for g in gaps if g.area > 0]
            gaps.sort(key=lambda g: g.area, reverse=True)

            isolated_to_place = [n for n in group.isolated_names if n not in self.lsi_names]
            for name in isolated_to_place:
                inst = self.design.get_instance(name)
                def_ = self.design.get_def(inst.reference)
                if not def_:
                    continue

                temp_pose = InstancePose(x=0, y=0, z=0, orientation=inst.pose.orientation, flip=inst.pose.flip)
                inst_aabb = GeometryEngine.compute_global_aabb(temp_pose, def_)
                inst_w = inst_aabb.width
                inst_h = inst_aabb.height

                placed = False
                for gap in gaps:
                    if gap.width >= inst_w and gap.height >= inst_h:
                        inst.pose.x = gap.x1 + (gap.width - inst_w) / 2.0
                        inst.pose.y = gap.y1 + (gap.height - inst_h) / 2.0
                        placed = True
                        gaps.remove(gap)
                        break

                if not placed:
                    self._place_at_boundary(inst, def_, group.aabb)

    def _place_isolated_in_empty_group(self, group):
        """Distribute isolated instances evenly in an empty group."""
        names = [n for n in group.isolated_names if n not in self.lsi_names]
        if not names:
            return

        n = len(names)
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)

        cell_w = group.aabb.width / cols
        cell_h = group.aabb.height / rows

        idx = 0
        for r in range(rows):
            for c in range(cols):
                if idx >= len(names):
                    break
                name = names[idx]
                inst = self.design.get_instance(name)
                def_ = self.design.get_def(inst.reference)
                if def_:
                    inst.pose.x = group.aabb.x1 + c * cell_w + cell_w / 2.0 - def_.width / 2.0
                    inst.pose.y = group.aabb.y1 + r * cell_h + cell_h / 2.0 - def_.height / 2.0
                idx += 1

    def _place_at_boundary(self, inst, def_, aabb):
        """Place an instance at the boundary of an AABB."""
        inst.pose.x = aabb.x1
        inst.pose.y = aabb.y1

    # ------------------------------------------------------------------
    # Step 7: Design merge
    # ------------------------------------------------------------------

    def _step7_design_merge(self) -> None:
        """Merge groups that have cross-group D2D connections."""
        cross_group_conns = []
        for conn in self.design.d2d_connections:
            if conn.has_lsi:
                continue
            src_group = self.instance_to_group.get(conn.source_inst)
            tgt_group = self.instance_to_group.get(conn.target_inst)
            if src_group is not None and tgt_group is not None and src_group != tgt_group:
                cross_group_conns.append((conn, src_group, tgt_group))

        if not cross_group_conns:
            return

        target_group_shifts = {}
        for conn, src_group, tgt_group in cross_group_conns:
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

            src_aabb = src_inst.global_aabb(src_def)
            tgt_aabb = tgt_inst.global_aabb(tgt_def)

            src_edge = self._get_phy_edge_global(src_inst, src_def, conn.source_ip)
            tgt_edge = self._get_phy_edge_global(tgt_inst, tgt_def, conn.target_ip)

            dx, dy = 0.0, 0.0
            if src_edge == 'right' and tgt_edge == 'left':
                dx = src_aabb.x2 - tgt_aabb.x1
                dy = src_ip_pos[1] - tgt_ip_pos[1]
            elif src_edge == 'left' and tgt_edge == 'right':
                dx = src_aabb.x1 - tgt_aabb.x2
                dy = src_ip_pos[1] - tgt_ip_pos[1]
            elif src_edge == 'top' and tgt_edge == 'bottom':
                dx = src_ip_pos[0] - tgt_ip_pos[0]
                dy = src_aabb.y2 - tgt_aabb.y1
            elif src_edge == 'bottom' and tgt_edge == 'top':
                dx = src_ip_pos[0] - tgt_ip_pos[0]
                dy = src_aabb.y1 - tgt_aabb.y2
            else:
                dx = src_ip_pos[0] - tgt_ip_pos[0]
                dy = src_ip_pos[1] - tgt_ip_pos[1]
                if src_aabb.x2 <= tgt_aabb.x1:
                    dx = src_aabb.x2 - tgt_aabb.x1
                elif tgt_aabb.x2 <= src_aabb.x1:
                    dx = src_aabb.x1 - tgt_aabb.x2
                else:
                    dx = 0

            if tgt_group not in target_group_shifts:
                target_group_shifts[tgt_group] = []
            target_group_shifts[tgt_group].append((dx, dy))

        for tgt_group, shifts in target_group_shifts.items():
            if not shifts:
                continue
            avg_dx = sum(s[0] for s in shifts) / len(shifts)
            avg_dy = sum(s[1] for s in shifts) / len(shifts)

            for inst_name in self._get_group_instances(tgt_group):
                inst = self.design.get_instance(inst_name)
                if inst:
                    inst.pose.x += avg_dx
                    inst.pose.y += avg_dy

    def _get_group_instances(self, group_idx: int) -> List[str]:
        """Return all instance names in a group."""
        group = self.groups[group_idx]
        names = []
        if group.dominant_name:
            names.append(group.dominant_name)
        names.extend(group.slave_names)
        names.extend(group.isolated_names)
        return names

    # ------------------------------------------------------------------
    # Step 8: Merged design placement
    # ------------------------------------------------------------------

    def _step8_center_design(self) -> None:
        """Translate all instances so MBR center aligns with Interposer center."""
        mbr = self.design.mbr_of_instances()
        mbr_cx = (mbr.x1 + mbr.x2) / 2.0
        mbr_cy = (mbr.y1 + mbr.y2) / 2.0

        ip_def = self.design.get_def(INTERPOSER_REF)
        if ip_def:
            ref_cx = ip_def.width / 2.0
            ref_cy = ip_def.height / 2.0
        else:
            ref_cx = mbr_cx
            ref_cy = mbr_cy

        dx = ref_cx - mbr_cx
        dy = ref_cy - mbr_cy

        for inst in self.design.instances:
            if inst.reference == INTERPOSER_REF:
                continue
            inst.pose.x += dx
            inst.pose.y += dy

    # ------------------------------------------------------------------
    # Overlap and spacing resolution
    # ------------------------------------------------------------------

    def _resolve_overlaps_and_spacing(self, max_iterations: int = 100) -> None:
        """Iteratively resolve overlaps and spacing violations.

        Skips pairs with direct D2D connections to preserve abutment.
        """
        d2d_pairs = set()
        for conn in self.design.d2d_connections:
            if conn.has_lsi:
                continue
            d2d_pairs.add((conn.source_inst, conn.target_inst))
            d2d_pairs.add((conn.target_inst, conn.source_inst))

        for _ in range(max_iterations):
            any_violation = False
            instances = [inst for inst in self.design.instances if inst.reference != INTERPOSER_REF]

            for i in range(len(instances)):
                def_i = self.design.get_def(instances[i].reference)
                if not def_i:
                    continue
                margin_i = max(def_i.seal_ring) + max(def_i.scribe_line)
                aabb_i = instances[i].global_aabb(def_i).inflate(margin_i)
                z_i_start = instances[i].pose.z
                z_i_end = z_i_start + def_i.thickness

                for j in range(i + 1, len(instances)):
                    def_j = self.design.get_def(instances[j].reference)
                    if not def_j:
                        continue

                    if (instances[i].name, instances[j].name) in d2d_pairs:
                        continue

                    z_j_start = instances[j].pose.z
                    z_j_end = z_j_start + def_j.thickness
                    if z_i_end <= z_j_start or z_j_end <= z_i_start:
                        continue
                    margin_j = max(def_j.seal_ring) + max(def_j.scribe_line)
                    aabb_j = instances[j].global_aabb(def_j).inflate(margin_j)

                    if aabb_i.overlaps(aabb_j):
                        any_violation = True
                        dx1, dy1, dx2, dy2 = GeometryEngine.resolve_overlap(aabb_i, aabb_j)
                        instances[i].pose.x += dx1
                        instances[i].pose.y += dy1
                        instances[j].pose.x += dx2
                        instances[j].pose.y += dy2

            if not any_violation:
                break
