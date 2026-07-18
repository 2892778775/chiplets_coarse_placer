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
            base_ref = self.design.reference_def_name()
            ip_def = self.design.get_def(base_ref) if base_ref else None
            if ip_def:
                ref_cx = ip_def.width / 2.0
                ref_cy = ip_def.height / 2.0
                mbr_cx = (mbr.x1 + mbr.x2) / 2.0
                mbr_cy = (mbr.y1 + mbr.y2) / 2.0
                dx = ref_cx - mbr_cx
                dy = ref_cy - mbr_cy
                for inst in self.design.instances:
                    if not self.design.is_base_instance(inst):
                        inst.pose.x += dx
                        inst.pose.y += dy
            return

        self._construct_generic_placement()

    def _get_max_margin(self) -> float:
        """Compute the maximum margin (seal_ring + scribe_line) across all chiplet types."""
        max_margin = 0.0
        base_ref = self.design.reference_def_name()
        for def_ in self.design.chiplet_defs.values():
            if def_.name == base_ref:
                continue
            margin = max(def_.seal_ring) + max(def_.scribe_line)
            if margin > max_margin:
                max_margin = margin
        return max_margin

    def _construct_generic_placement(self) -> None:
        """Generic fallback: pack by Z layer side by side."""
        z_layers = {}
        for inst in self.design.instances:
            if self.design.is_base_instance(inst):
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
            if not self.design.is_base_instance(inst) and inst.flexibility.status != "fixed"
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

    def _base_ref_name(self) -> Optional[str]:
        """Name of the base (reference layer) def: 'Interposer' or largest-area def (e.g. 'RW')."""
        return self.design.reference_def_name()

    def _is_base(self, inst: ChipletInst) -> bool:
        """Check if an instance is on the base/reference layer."""
        return self.design.is_base_instance(inst)

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
        self.bridge_map: Dict[str, Tuple[str, str]] = {}  # lsi_name -> (inst_a, inst_b)
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
        """Identify dominant instances, slaves, LSI bridges, and isolated instances."""
        degrees: Dict[str, int] = {}
        for inst in self.design.instances:
            if self._is_base(inst):
                continue
            degrees[inst.name] = 0

        # LSI bridges appear in the lsi_inst slot of a connection, never as
        # source/target. Identify them first so they are never mistaken for
        # slaves or isolated chiplets.
        self.lsi_names = set()
        self.bridge_map = {}
        for conn in self.design.d2d_connections:
            if conn.lsi_inst:
                self.lsi_names.add(conn.lsi_inst)
                self.bridge_map[conn.lsi_inst] = (conn.source_inst, conn.target_inst)
        for name in degrees:
            inst = self.design.get_instance(name)
            if inst and "LSI" in inst.reference.upper():
                self.lsi_names.add(name)

        for conn in self.design.d2d_connections:
            if conn.source_inst in degrees:
                degrees[conn.source_inst] += 1
            if conn.target_inst in degrees:
                degrees[conn.target_inst] += 1
            if conn.lsi_inst and conn.lsi_inst in degrees:
                degrees[conn.lsi_inst] += 2

        non_lsi_degrees = {n: d for n, d in degrees.items() if n not in self.lsi_names}
        max_degree = max(non_lsi_degrees.values()) if non_lsi_degrees else 0

        has_soc = any(
            any(kw in self.design.get_instance(name).reference.upper() for kw in SOC_KEYWORDS)
            for name in non_lsi_degrees if self.design.get_instance(name)
        )

        self.dominant_names = set()
        for name, deg in non_lsi_degrees.items():
            inst = self.design.get_instance(name)
            if not inst:
                continue
            ref_upper = inst.reference.upper()
            if any(kw in ref_upper for kw in SOC_KEYWORDS):
                self.dominant_names.add(name)
            elif not has_soc and deg == max_degree and max_degree > 0:
                self.dominant_names.add(name)

        if not self.dominant_names:
            for name, deg in non_lsi_degrees.items():
                if deg > 0:
                    self.dominant_names.add(name)

        self.slave_map = {name: [] for name in self.dominant_names}
        for name, deg in non_lsi_degrees.items():
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
        for name, deg in non_lsi_degrees.items():
            if name in self.dominant_names:
                continue
            if any(name in slaves for slaves in self.slave_map.values()):
                continue
            if deg == 0:
                self.isolated_names.add(name)

    # ------------------------------------------------------------------
    # Step 2: Design partition
    # ------------------------------------------------------------------

    def _step2_design_partition(self) -> None:
        """Partition the base layer into N groups for N dominant instances."""
        # Reference size: the base layer def ("Interposer"), or — for CoW designs
        # without an Interposer def — the largest-area chiplet def (e.g. "RW").
        # Using a hardcoded default here would pick a wrong aspect ratio and
        # hence a wrong partition direction.
        ip_def = self.design.get_def(INTERPOSER_REF)
        if not ip_def:
            ip_def = max(
                (d for d in self.design.chiplet_defs.values()),
                key=lambda d: d.width * d.height,
                default=None
            )
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
                    group_cy = (group.aabb.y1 + group.aabb.y2) / 2.0
                    partner_cy = (partner_group.aabb.y1 + partner_group.aabb.y2) / 2.0
                    # Choose the axis with the larger center separation so that
                    # vertically partitioned designs resolve to top/bottom edges.
                    if abs(partner_cy - group_cy) > abs(partner_cx - group_cx):
                        target_edge = 'top' if partner_cy > group_cy else 'bottom'
                    else:
                        target_edge = 'right' if partner_cx > group_cx else 'left'

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

            if partner_g_idx is not None and g_idx is not None and partner_g_idx != g_idx:
                # Cross-group connection: the PHY must face the partner group.
                # Compare both axes so vertically partitioned designs (CoW)
                # resolve to top/bottom instead of always left/right.
                partner_group = self.groups[partner_g_idx]
                grp = self.groups[g_idx]
                group_cx = (grp.aabb.x1 + grp.aabb.x2) / 2.0
                partner_cx = (partner_group.aabb.x1 + partner_group.aabb.x2) / 2.0
                group_cy = (grp.aabb.y1 + grp.aabb.y2) / 2.0
                partner_cy = (partner_group.aabb.y1 + partner_group.aabb.y2) / 2.0
                if abs(partner_cy - group_cy) > abs(partner_cx - group_cx):
                    desired[ip_name] = 'top' if partner_cy > group_cy else 'bottom'
                else:
                    desired[ip_name] = 'right' if partner_cx > group_cx else 'left'
            elif conn.has_lsi:
                # Same-group LSI-bridged connection: do NOT constrain the dominant
                # orientation. Slave sides adapt later to the dominant's
                # transformed PHY edges, so the dominant only needs to satisfy
                # its cross-group PHY directions.
                continue
            elif partner_g_idx is None or g_idx is None:
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
            else:
                # Same-group direct (non-bridged) connection: preserve the
                # designer-intended PHY edge from the omap.
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

        if desired:
            cross_conns = self._cross_group_conns_of(inst_name)
            return self._score_dominant_orientations(inst_name, desired, cross_conns)
        return 'R0', 'None'

    def _cross_group_conns_of(self, inst_name: str) -> List:
        """Return all cross-group D2D connections involving inst_name."""
        g_idx = self.instance_to_group.get(inst_name)
        if g_idx is None:
            return []
        result = []
        for conn in self.design.d2d_connections:
            if conn.source_inst == inst_name:
                partner = conn.target_inst
            elif conn.target_inst == inst_name:
                partner = conn.source_inst
            else:
                continue
            p_g = self.instance_to_group.get(partner)
            if p_g is not None and p_g != g_idx:
                result.append(conn)
        return result

    def _score_dominant_orientations(self, inst_name: str, desired: Dict[str, str],
                                     cross_conns: List) -> Tuple[str, str]:
        """Score (orient, flip) candidates by desired-edge matches minus crossing penalty.

        The crossing penalty prevents X-crossed D2D pairs between two dominants:
        when two or more cross-group connections link the same pair of dies, the
        PHY order along the abutment axis must be preserved (e.g. SoIC_BD_1 takes
        R180 rather than MX so that uciesp_1_0<->uciesp_1_1 pairs do not cross).
        """
        inst = self.design.get_instance(inst_name)
        def_ = self.design.get_def(inst.reference)
        if not def_:
            return 'R0', 'None'

        vertical_abut = any(d in ('top', 'bottom') for d in desired.values())

        best_score = float('-inf')
        best_orient, best_flip = 'R0', 'None'

        for orient in ORIENTATIONS:
            for flip in FLIPS:
                score = 0.0
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

                if len(cross_conns) >= 2:
                    pts = []  # (this_phy_along_axis, partner_phy_along_axis)
                    for conn in cross_conns:
                        if conn.source_inst == inst_name:
                            s_ip, t_ip, partner_name = conn.source_ip, conn.target_ip, conn.target_inst
                        else:
                            s_ip, t_ip, partner_name = conn.target_ip, conn.source_ip, conn.source_inst
                        partner = self.design.get_instance(partner_name)
                        p_def = self.design.get_def(partner.reference) if partner else None
                        if not p_def:
                            continue
                        cand_pose = InstancePose(x=0, y=0, z=0, orientation=orient, flip=flip)
                        s_pos = GeometryEngine.compute_ip_global_position(cand_pose, def_, s_ip)
                        t_pos = partner.global_ip_position(p_def, t_ip)
                        if s_pos and t_pos:
                            if vertical_abut:
                                pts.append((s_pos[0], t_pos[0]))
                            else:
                                pts.append((s_pos[1], t_pos[1]))
                    for i in range(len(pts)):
                        for j in range(i + 1, len(pts)):
                            o_src = pts[i][0] - pts[j][0]
                            o_tgt = pts[i][1] - pts[j][1]
                            if o_src != 0 and o_tgt != 0 and (o_src < 0) != (o_tgt < 0):
                                score += ORIENT_SCORE_OPPOSITE  # crossing pair

                if score > best_score:
                    best_score = score
                    best_orient, best_flip = orient, flip

        return best_orient, best_flip

    # ------------------------------------------------------------------
    # Step 5: Slave instances placement
    # ------------------------------------------------------------------

    def _step5_slave_placement(self) -> None:
        """Place slave instances to abut and PHY-align with their dominant instance."""
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

                self._place_slave_for_connection(dominant, dominant_def, slave, slave_def, conns)

    def _place_slave_for_connection(self, dominant, dominant_def, slave, slave_def, conns):
        """Choose orientation/flip and position for a slave via PHY alignment.

        For every (orientation, flip) candidate the slave is placed so that it
        abuts the dominant on the side of the dominant's PHY edge, with PHY
        centers aligned exactly along the abutment axis. The candidate with the
        smallest total Manhattan PHY-to-PHY distance wins. This reproduces the
        mirror conventions of manual CoW placements (e.g. HBM stacks facing a
        SoIC take MY / R180 depending on which side they sit on) without any
        hard-coded per-type rules.
        """
        primary = conns[0]
        dom_ip = primary.source_ip if primary.source_inst == dominant.name else primary.target_ip
        dom_edge = self._get_phy_edge_global(dominant, dominant_def, dom_ip) or 'right'
        slv_ip_primary = primary.target_ip if primary.target_inst == slave.name else primary.source_ip

        best = None  # (total_dist, orient, flip, x, y)
        for orient in ORIENTATIONS:
            for flip in FLIPS:
                # PHY orientation filter: the PHY's long axis must stay parallel
                # to the abutment edge (tall PHY for left/right abutment, wide
                # PHY for top/bottom). Rotating a tall HBM PHY by 90 degrees
                # would shorten the PHY distance but break bump-row alignment
                # and stack packing, so those candidates are rejected upfront.
                if not self._phy_axis_compatible(slave, slave_def, slv_ip_primary,
                                                 orient, flip, dom_edge):
                    continue
                cand = self._place_slave_candidate(dominant, dominant_def, slave, slave_def,
                                                   conns, dom_edge, orient, flip)
                if cand is None:
                    continue
                if best is None or cand[0] < best[0] - 1e-9:
                    best = cand

        if best is None:
            # No axis-compatible candidate: retry without the filter.
            for orient in ORIENTATIONS:
                for flip in FLIPS:
                    cand = self._place_slave_candidate(dominant, dominant_def, slave, slave_def,
                                                       conns, dom_edge, orient, flip)
                    if cand is None:
                        continue
                    if best is None or cand[0] < best[0] - 1e-9:
                        best = cand

        if best is None:
            self._place_slave_near_dominant(dominant, dominant_def, slave, slave_def)
            return

        _, orient, flip, x, y = best

        # For LSI-bridged top/bottom abutment, exact abutment can create corner
        # margin clashes with side slaves (e.g. HBM stacks whose combined height
        # equals the dominant's height). H4 abutment is relaxed for bridged
        # connections, so inset the slave just enough to clear the margins.
        if dom_edge in ('top', 'bottom') and any(c.has_lsi for c in conns):
            x, y = self._inset_for_corner_clearance(slave, slave_def, orient, flip,
                                                    x, y, dom_edge, conns)

        slave.pose.orientation = orient
        slave.pose.flip = flip
        slave.pose.x = x
        slave.pose.y = y

    def _inset_for_corner_clearance(self, slave, slave_def, orient, flip,
                                    x, y, dom_edge, conns) -> Tuple[float, float]:
        """Inset a top/bottom slave away from the dominant until margin clashes clear."""
        m_s = max(slave_def.seal_ring) + max(slave_def.scribe_line)
        partners = set()
        for c in conns:
            partners.add(c.source_inst)
            partners.add(c.target_inst)

        def clash_margin(px: float, py: float) -> float:
            pose = InstancePose(x=px, y=py, z=slave.pose.z, orientation=orient, flip=flip)
            aabb = GeometryEngine.compute_global_aabb(pose, slave_def)
            sz0, sz1 = slave.pose.z, slave.pose.z + slave_def.thickness
            for other in self.design.instances:
                if other.name == slave.name or other.name in partners or self._is_base(other):
                    continue
                od = self.design.get_def(other.reference)
                if not od:
                    continue
                oz0, oz1 = other.pose.z, other.pose.z + od.thickness
                if sz1 <= oz0 or oz1 <= sz0:
                    continue
                m_o = max(od.seal_ring) + max(od.scribe_line)
                if aabb.inflate(m_s + m_o + SPACING_EPSILON).overlaps(other.global_aabb(od)):
                    return m_o
            return -1.0

        m_clash = clash_margin(x, y)
        if m_clash < 0:
            return x, y
        inset = m_s + m_clash + SPACING_EPSILON
        if dom_edge == 'top':
            y += inset
        else:
            y -= inset
        return x, y

    def _phy_axis_compatible(self, slave, slave_def, slv_ip, orient, flip, dom_edge) -> bool:
        """Check that the slave PHY's long axis stays parallel to the abutment edge."""
        pose0 = InstancePose(x=0, y=0, z=slave.pose.z, orientation=orient, flip=flip)
        ip_aabb = GeometryEngine.compute_ip_global_aabb(pose0, slave_def, slv_ip)
        if not ip_aabb:
            return True
        if dom_edge in ('left', 'right'):
            return ip_aabb.height >= ip_aabb.width
        return ip_aabb.width >= ip_aabb.height

    def _place_slave_candidate(self, dominant, dominant_def, slave, slave_def,
                               conns, dom_edge, orient, flip):
        """Evaluate one (orient, flip) candidate: abut + align, return PHY distance."""
        pose0 = InstancePose(x=0, y=0, z=slave.pose.z, orientation=orient, flip=flip)
        slave_aabb0 = GeometryEngine.compute_global_aabb(pose0, slave_def)
        dom_aabb = dominant.global_aabb(dominant_def)

        primary = conns[0]
        dom_ip = primary.source_ip if primary.source_inst == dominant.name else primary.target_ip
        slv_ip = primary.target_ip if primary.target_inst == slave.name else primary.source_ip

        dom_ip_pos = dominant.global_ip_position(dominant_def, dom_ip)
        slv_ip_pos0 = GeometryEngine.compute_ip_global_position(pose0, slave_def, slv_ip)
        if not dom_ip_pos or not slv_ip_pos0:
            return None

        if dom_edge == 'left':
            x = dom_aabb.x1 - slave_aabb0.width
            y = dom_ip_pos[1] - slv_ip_pos0[1]
        elif dom_edge == 'right':
            x = dom_aabb.x2
            y = dom_ip_pos[1] - slv_ip_pos0[1]
        elif dom_edge == 'top':
            x = dom_ip_pos[0] - slv_ip_pos0[0]
            y = dom_aabb.y2
        else:  # 'bottom'
            x = dom_ip_pos[0] - slv_ip_pos0[0]
            y = dom_aabb.y1 - slave_aabb0.height

        # Total Manhattan PHY distance over all connections of this slave.
        total = 0.0
        pose = InstancePose(x=x, y=y, z=slave.pose.z, orientation=orient, flip=flip)
        for conn in conns:
            d_ip = conn.source_ip if conn.source_inst == dominant.name else conn.target_ip
            s_ip = conn.target_ip if conn.target_inst == slave.name else conn.source_ip
            d_pos = dominant.global_ip_position(dominant_def, d_ip)
            s_pos = GeometryEngine.compute_ip_global_position(pose, slave_def, s_ip)
            if not d_pos or not s_pos:
                return None
            total += abs(d_pos[0] - s_pos[0]) + abs(d_pos[1] - s_pos[1])

        return (total, orient, flip, x, y)

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
        """Place isolated instances (IOD, IPD, DUMMY) in available free space.

        Margin-aware corner search over the base-layer area: candidate positions
        are generated from placed instances' AABB edges and the base boundary;
        the first candidate that clears all margin requirements at the
        instance's Z layer wins (preferring the instance's own group region).
        """
        # Candidate search area. Step 8 translates the whole design by
        # (base_center - MBR center); to keep candidates inside the base
        # layer after that translation, pre-offset the base footprint by
        # (anchored MBR center - base center) and hard-constrain the search
        # to the resulting effective area. The anchored MBR only includes
        # instances that are already placed (dominants, slaves, LSI bridges);
        # unplaced isolated instances still sit at their parser-default pose
        # and must not pollute it.
        anchored_boxes = []
        for inst in self.design.instances:
            if inst.name in self.isolated_names or self._is_base(inst):
                continue
            ad = self.design.get_def(inst.reference)
            if ad:
                anchored_boxes.append(inst.global_aabb(ad))

        base_inst = self.design.base_instance()
        base_def = self.design.get_def(base_inst.reference) if base_inst else None
        if base_inst and base_def and anchored_boxes:
            base_aabb = base_inst.global_aabb(base_def)
            ax1 = min(b.x1 for b in anchored_boxes)
            ay1 = min(b.y1 for b in anchored_boxes)
            ax2 = max(b.x2 for b in anchored_boxes)
            ay2 = max(b.y2 for b in anchored_boxes)
            dx = (ax1 + ax2) / 2.0 - (base_aabb.x1 + base_aabb.x2) / 2.0
            dy = (ay1 + ay2) / 2.0 - (base_aabb.y1 + base_aabb.y2) / 2.0
            area = AABB(base_aabb.x1 + dx, base_aabb.y1 + dy,
                        base_aabb.x2 + dx, base_aabb.y2 + dy)
        else:
            base_ref = self._base_ref_name()
            base_def = self.design.get_def(base_ref) if base_ref else None
            if base_def:
                area = AABB(0.0, 0.0, base_def.width, base_def.height)
            else:
                mbr = self.design.mbr_of_instances()
                slack = 60000.0
                area = AABB(mbr.x1 - slack, mbr.y1 - slack, mbr.x2 + slack, mbr.y2 + slack)

        # Isolated instances placed during this step become obstacles for the
        # remaining ones; unplaced ones (parser-default pose) are ignored.
        self._isolated_placed = set()

        for group in self.groups:
            if not group.isolated_names:
                continue

            isolated_to_place = [n for n in group.isolated_names if n not in self.lsi_names]
            if not isolated_to_place:
                continue

            if not group.dominant_name and not group.slave_names:
                self._place_isolated_in_empty_group(group)
                continue

            for name in isolated_to_place:
                inst = self.design.get_instance(name)
                def_ = self.design.get_def(inst.reference)
                if not def_:
                    continue
                if not self._place_isolated_margin_aware(inst, def_, area, group.aabb):
                    self._place_at_boundary(inst, def_, area)
                self._isolated_placed.add(name)

    def _place_isolated_margin_aware(self, inst, def_, area: AABB, group_aabb: AABB) -> bool:
        """First-fit search of margin-clear candidate positions for an isolated instance."""
        w, h = def_.width, def_.height
        m_i = max(def_.seal_ring) + max(def_.scribe_line)

        # Collect placed instances on overlapping Z layers. Isolated
        # instances not yet placed in this step are still at their
        # parser-default pose and are skipped.
        placed_set = getattr(self, "_isolated_placed", set())
        placed = []
        z0, z1 = inst.pose.z, inst.pose.z + def_.thickness
        for other in self.design.instances:
            if other.name == inst.name or self._is_base(other):
                continue
            if other.name in self.isolated_names and other.name not in placed_set:
                continue
            od = self.design.get_def(other.reference)
            if not od:
                continue
            oz0, oz1 = other.pose.z, other.pose.z + od.thickness
            if z1 <= oz0 or oz1 <= z0:
                continue
            placed.append((other.global_aabb(od), max(od.seal_ring) + max(od.scribe_line)))

        xs = {area.x1 + m_i, area.x2 - m_i - w}
        ys = {area.y1 + m_i, area.y2 - m_i - h}
        for a, m_o in placed:
            mm = m_i + m_o + SPACING_EPSILON
            xs.update([a.x1 - mm - w, a.x2 + mm, a.x1, a.x2 - w])
            ys.update([a.y1 - mm - h, a.y2 + mm, a.y1, a.y2 - h])

        best = None  # (in_group, out_of_base, y, x)
        for cx in xs:
            for cy in ys:
                if cx < area.x1 or cy < area.y1 or cx + w > area.x2 or cy + h > area.y2:
                    continue
                cand = AABB(cx, cy, cx + w, cy + h)
                ok = True
                for a, m_o in placed:
                    mm = m_i + m_o + SPACING_EPSILON
                    if cand.inflate(mm).overlaps(a):
                        ok = False
                        break
                if not ok:
                    continue
                in_group = (group_aabb.x1 <= cx and cx + w <= group_aabb.x2 and
                            group_aabb.y1 <= cy and cy + h <= group_aabb.y2)
                best_key = (0 if in_group else 1, cy, cx)
                if best is None or best_key < best[0]:
                    best = (best_key, cx, cy)

        if best is None:
            return False
        _, bx, by = best
        inst.pose.x = bx
        inst.pose.y = by
        return True

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
                    self._isolated_placed.add(name)
                idx += 1

    def _place_at_boundary(self, inst, def_, aabb):
        """Place an instance at the boundary of an AABB."""
        inst.pose.x = aabb.x1
        inst.pose.y = aabb.y1

    # ------------------------------------------------------------------
    # Step 7: Design merge
    # ------------------------------------------------------------------

    def _step7_design_merge(self) -> None:
        """Merge groups that have cross-group D2D connections.

        LSI-bridged connections are enforced as well: the LSI sits at a lower
        Z layer (z=250) and does not affect the abutment requirement of the two
        top dies (z=500), which must still abut directly.
        """
        cross_group_conns = []
        for conn in self.design.d2d_connections:
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

            # Clearance between merged groups so that margin-inflated AABBs of
            # cross-group neighbor chiplets (e.g. an HBM stack reaching the top
            # of SoIC_BD_0 vs the bottom of SoIC_BD_1) do not violate H5.
            clearance = self._group_clearance(src_group, tgt_group)

            dx, dy = 0.0, 0.0
            if src_edge == 'right' and tgt_edge == 'left':
                dx = src_aabb.x2 - tgt_aabb.x1 + clearance
                dy = src_ip_pos[1] - tgt_ip_pos[1]
            elif src_edge == 'left' and tgt_edge == 'right':
                dx = src_aabb.x1 - tgt_aabb.x2 - clearance
                dy = src_ip_pos[1] - tgt_ip_pos[1]
            elif src_edge == 'top' and tgt_edge == 'bottom':
                dx = src_ip_pos[0] - tgt_ip_pos[0]
                dy = src_aabb.y2 - tgt_aabb.y1 + clearance
            elif src_edge == 'bottom' and tgt_edge == 'top':
                dx = src_ip_pos[0] - tgt_ip_pos[0]
                dy = src_aabb.y1 - tgt_aabb.y2 - clearance
            elif src_edge in ('top', 'bottom') and tgt_edge in ('top', 'bottom'):
                # Same-axis edges (e.g. top-top): align PHY x and stack vertically
                # following the current relative position of the two groups.
                dx = src_ip_pos[0] - tgt_ip_pos[0]
                if tgt_aabb.center[1] >= src_aabb.center[1]:
                    dy = src_aabb.y2 - tgt_aabb.y1 + clearance
                else:
                    dy = src_aabb.y1 - tgt_aabb.y2 - clearance
            elif src_edge in ('left', 'right') and tgt_edge in ('left', 'right'):
                # Same-axis edges (e.g. left-left): align PHY y and abut horizontally.
                dy = src_ip_pos[1] - tgt_ip_pos[1]
                if tgt_aabb.center[0] >= src_aabb.center[0]:
                    dx = src_aabb.x2 - tgt_aabb.x1 + clearance
                else:
                    dx = src_aabb.x1 - tgt_aabb.x2 - clearance
            else:
                dx = src_ip_pos[0] - tgt_ip_pos[0]
                dy = src_ip_pos[1] - tgt_ip_pos[1]

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

    def _group_clearance(self, g1: int, g2: int) -> float:
        """Max margin (seal_ring + scribe_line) across both groups + epsilon."""
        margin = 0.0
        for g in (g1, g2):
            for name in self._get_group_instances(g):
                inst = self.design.get_instance(name)
                def_ = self.design.get_def(inst.reference) if inst else None
                if def_:
                    margin = max(margin, max(def_.seal_ring) + max(def_.scribe_line))
        return margin + SPACING_EPSILON

    # ------------------------------------------------------------------
    # Step 8: Merged design placement
    # ------------------------------------------------------------------

    def _step8_center_design(self) -> None:
        """Translate all instances so the MBR center aligns with the base layer center.

        Reference center priority: actual base instance AABB center (u_Interposer
        or uRW) > base def size / 2 > translate MBR to positive enclosure margin.
        """
        mbr = self.design.mbr_of_instances()
        mbr_cx = (mbr.x1 + mbr.x2) / 2.0
        mbr_cy = (mbr.y1 + mbr.y2) / 2.0

        ref_cx = ref_cy = None
        base_inst = self.design.base_instance()
        if base_inst:
            base_def = self.design.get_def(base_inst.reference)
            if base_def:
                ref_cx, ref_cy = base_inst.global_aabb(base_def).center

        if ref_cx is None:
            base_ref = self._base_ref_name()
            ip_def = self.design.get_def(base_ref) if base_ref else None
            if ip_def:
                ref_cx = ip_def.width / 2.0
                ref_cy = ip_def.height / 2.0

        if ref_cx is None:
            # No base layer at all: shift the design into positive coordinates.
            dx = self.enclosure - mbr.x1
            dy = self.enclosure - mbr.y1
        else:
            dx = ref_cx - mbr_cx
            dy = ref_cy - mbr_cy

        for inst in self.design.instances:
            if self._is_base(inst):
                continue
            inst.pose.x += dx
            inst.pose.y += dy

    # ------------------------------------------------------------------
    # Overlap and spacing resolution
    # ------------------------------------------------------------------

    def _resolve_overlaps_and_spacing(self, max_iterations: int = 100) -> None:
        """Iteratively resolve overlaps and spacing violations.

        Movement policy:
        - Anchored instances (dominants, D2D-connected slaves) never move:
          their positions encode exact abutment / PHY alignment.
        - LSI bridges never move: their centers must stay at the exact D2D
          PHY midpoint (hard rule H7).
        - Only floating (isolated) instances are moved to resolve violations.
        - Base-layer instances (Interposer / RW) are excluded entirely:
          lower-Z chiplets such as LSI bridges are embedded by design.
        """
        d2d_pairs = set()
        connected = set()
        for conn in self.design.d2d_connections:
            d2d_pairs.add((conn.source_inst, conn.target_inst))
            d2d_pairs.add((conn.target_inst, conn.source_inst))
            connected.add(conn.source_inst)
            connected.add(conn.target_inst)

        movable = set()
        for name in self.isolated_names:
            if name not in connected and name not in self.lsi_names:
                movable.add(name)

        for _ in range(max_iterations):
            any_violation = False
            instances = [inst for inst in self.design.instances if not self._is_base(inst)]

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
                        i_movable = instances[i].name in movable
                        j_movable = instances[j].name in movable
                        if not i_movable and not j_movable:
                            continue
                        any_violation = True
                        dx1, dy1, dx2, dy2 = GeometryEngine.resolve_overlap(aabb_i, aabb_j)
                        if i_movable and j_movable:
                            instances[i].pose.x += dx1
                            instances[i].pose.y += dy1
                            instances[j].pose.x += dx2
                            instances[j].pose.y += dy2
                        elif i_movable:
                            instances[i].pose.x += 2 * dx1
                            instances[i].pose.y += 2 * dy1
                        else:
                            instances[j].pose.x += 2 * dx2
                            instances[j].pose.y += 2 * dy2

            if not any_violation:
                break
