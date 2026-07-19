"""
Core data models for the 3D IC Chiplets Coarse-Placement System.

All geometric units are in microns (μm).
"""

from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set
import math


@dataclass
class AABB:
    """Axis-Aligned Bounding Box."""
    x1: float = 0.0
    y1: float = 0.0
    x2: float = 0.0
    y2: float = 0.0

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

    def contains(self, other: "AABB") -> bool:
        """Return True if this AABB fully contains other."""
        return (self.x1 <= other.x1 and self.y1 <= other.y1 and
                self.x2 >= other.x2 and self.y2 >= other.y2)

    def overlaps(self, other: "AABB", margin: float = 0.0) -> bool:
        """Return True if this AABB overlaps with other (with optional margin)."""
        return not (self.x2 + margin <= other.x1 or
                    other.x2 + margin <= self.x1 or
                    self.y2 + margin <= other.y1 or
                    other.y2 + margin <= self.y1)

    def distance_to(self, other: "AABB") -> float:
        """Minimum distance between two AABBs. Zero if overlapping."""
        dx = max(self.x1 - other.x2, other.x1 - self.x2, 0.0)
        dy = max(self.y1 - other.y2, other.y1 - self.y2, 0.0)
        return math.hypot(dx, dy)

    def inflate(self, dx: float, dy: Optional[float] = None) -> "AABB":
        """Return a new AABB inflated by dx, dy on all sides."""
        if dy is None:
            dy = dx
        return AABB(self.x1 - dx, self.y1 - dy, self.x2 + dx, self.y2 + dy)

    def copy(self) -> "AABB":
        return AABB(self.x1, self.y1, self.x2, self.y2)

    def __repr__(self) -> str:
        return f"AABB({self.x1:.1f}, {self.y1:.1f}, {self.x2:.1f}, {self.y2:.1f})"


@dataclass
class ObjectDef:
    """An object (IP) defined inside a chiplet, e.g., SOC_IP, HBM_IP."""
    name: str
    size: Tuple[float, float] = (0.0, 0.0)
    layer: List[Tuple[float, float]] = field(default_factory=list)

    @property
    def width(self) -> float:
        return self.size[0]

    @property
    def height(self) -> float:
        return self.size[1]


@dataclass
class ObjectMapEntry:
    """A single IP instance placed inside a chiplet (local coordinates)."""
    obj_type: str           # e.g., "SOC_IP"
    name: str               # e.g., "PHY0"
    loc_x: float = 0.0
    loc_y: float = 0.0
    orientation: str = "R0"

    def local_aabb(self, obj_size: Tuple[float, float]) -> AABB:
        """Return AABB in chiplet local coordinates."""
        w, h = obj_size
        return AABB(self.loc_x, self.loc_y, self.loc_x + w, self.loc_y + h)


@dataclass
class ChipletDef:
    """Definition of a chiplet type (from .3dbv file)."""
    name: str
    size: Tuple[float, float] = (0.0, 0.0)
    shrink: float = 1.0
    thickness: float = 0.0
    seal_ring: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    scribe_line: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    omap_file: Optional[str] = None
    dbo_file: Optional[str] = None
    object_defs: Dict[str, ObjectDef] = field(default_factory=dict)
    omap_entries: List[ObjectMapEntry] = field(default_factory=list)

    @property
    def width(self) -> float:
        return self.size[0]

    @property
    def height(self) -> float:
        return self.size[1]

    def get_object_size(self, obj_type: str) -> Tuple[float, float]:
        """Return size of an object by type name."""
        if obj_type in self.object_defs:
            return self.object_defs[obj_type].size
        return (0.0, 0.0)

    def local_aabb(self) -> AABB:
        """Return AABB in local coordinates (origin at 0,0)."""
        return AABB(0.0, 0.0, self.width, self.height)

    def expanded_aabb(self) -> AABB:
        """Return AABB expanded by seal_ring and scribe_line margins."""
        # seal_ring: [L, R, T, B]
        # scribe_line: [L, R, T, B]
        l_margin = self.seal_ring[0] + self.scribe_line[0]
        r_margin = self.seal_ring[1] + self.scribe_line[1]
        t_margin = self.seal_ring[2] + self.scribe_line[2]
        b_margin = self.seal_ring[3] + self.scribe_line[3]
        return AABB(-l_margin, -b_margin, self.width + r_margin, self.height + t_margin)


@dataclass
class InstancePose:
    """Pose (position + orientation + flip) of a chiplet instance."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    orientation: str = "R0"       # R0, R90, R180, R270
    flip: str = "None"          # None, MX, MY
    mz: bool = False            # False = face up, True = face down (Z-axis flip)

    def copy(self) -> "InstancePose":
        return InstancePose(self.x, self.y, self.z, self.orientation, self.flip, self.mz)


@dataclass
class Flexibility:
    """User-specified flexibility for an instance."""
    status: str = "placed"    # placed, fixed, floating
    shift: bool = True
    rotate: bool = True
    flip: bool = True
    resize: bool = False


@dataclass
class ChipletInst:
    """An instance of a chiplet placed in the design (from .3dbx Stack)."""
    name: str
    reference: str
    is_master: bool = True
    pose: InstancePose = field(default_factory=InstancePose)
    flexibility: Flexibility = field(default_factory=Flexibility)
    group: str = ""
    visible: bool = True

    def global_aabb(self, chiplet_def: ChipletDef) -> AABB:
        """Return AABB in global coordinates."""
        from .geometry import GeometryEngine
        return GeometryEngine.compute_global_aabb(self.pose, chiplet_def)

    def global_ip_position(self, chiplet_def: ChipletDef, ip_name: str) -> Optional[Tuple[float, float]]:
        """Return the global center position of an IP inside this instance."""
        from .geometry import GeometryEngine
        return GeometryEngine.compute_ip_global_position(self.pose, chiplet_def, ip_name)

    def global_ip_aabb(self, chiplet_def: ChipletDef, ip_name: str) -> Optional[AABB]:
        """Return the global AABB of an IP inside this instance."""
        from .geometry import GeometryEngine
        return GeometryEngine.compute_ip_global_aabb(self.pose, chiplet_def, ip_name)

    def __hash__(self) -> int:
        return hash(self.name)

    def __eq__(self, other) -> bool:
        if isinstance(other, ChipletInst):
            return self.name == other.name
        return False


@dataclass
class D2DConnection:
    """A D2D connection between two IP instances across chiplets.
    
    When lsi_inst is set, the connection is bridged through an LSI chiplet:
    source -> LSI -> target.
    """
    source_inst: str          # e.g., "u_SOC_0"
    source_ip: str            # e.g., "PHY0"
    target_inst: str          # e.g., "u_HBM_0"
    target_ip: str            # e.g., "PHY0"
    lsi_inst: str = ""        # e.g., "u_LSI1_0" (optional LSI bridge)
    is_external: bool = False

    @property
    def source_full(self) -> str:
        return f"{self.source_inst}.{self.source_ip}"
    
    @property
    def target_full(self) -> str:
        return f"{self.target_inst}.{self.target_ip}"
    
    @property
    def has_lsi(self) -> bool:
        return bool(self.lsi_inst)


@dataclass
class DesignModel:
    """In-memory representation of the entire design."""
    name: str = ""
    interposer: Optional[ChipletDef] = None
    chiplet_defs: Dict[str, ChipletDef] = field(default_factory=dict)
    instances: List[ChipletInst] = field(default_factory=list)
    d2d_connections: List[D2DConnection] = field(default_factory=list)
    # PI affinity: isolated instance name -> dominant instance name it belongs
    # to (from the optional LSI.PI file). Such instances must be placed inside
    # their dominant's footprint instead of in free space.
    pi_affinity: Dict[str, str] = field(default_factory=dict)
    raw_data: Dict = field(default_factory=dict)  # raw parsed YAML for export

    def get_instance(self, name: str) -> Optional[ChipletInst]:
        for inst in self.instances:
            if inst.name == name:
                return inst
        return None

    def get_def(self, ref: str) -> Optional[ChipletDef]:
        return self.chiplet_defs.get(ref)

    def instances_by_type(self, type_name: str) -> List[ChipletInst]:
        """Return instances whose reference contains the given type name."""
        return [inst for inst in self.instances if type_name in inst.reference]

    def reference_def_name(self) -> Optional[str]:
        """Name of the base (reference layer) chiplet definition.

        Returns "Interposer" when such a def exists (CoWoS-style input);
        otherwise falls back to the largest-area chiplet def, which in
        CoW (Chip-on-Wafer) designs is the reconstituted-wafer base (e.g. "RW").
        """
        if "Interposer" in self.chiplet_defs:
            return "Interposer"
        if not self.chiplet_defs:
            return None
        return max(
            self.chiplet_defs.values(),
            key=lambda d: d.width * d.height
        ).name

    def is_base_instance(self, inst: ChipletInst) -> bool:
        """True if the instance belongs to the base (reference) layer."""
        ref = self.reference_def_name()
        return ref is not None and inst.reference == ref

    def base_instance(self) -> Optional[ChipletInst]:
        """Return the first base-layer instance, if any."""
        for inst in self.instances:
            if self.is_base_instance(inst):
                return inst
        return None

    def mbr_of_instances(self, exclude_refs: Optional[Set[str]] = None) -> AABB:
        """Compute MBR of all instances (excluding the base layer and optional refs)."""
        if exclude_refs is None:
            base = self.reference_def_name()
            exclude_refs = {base} if base else set()
        bboxes = []
        for inst in self.instances:
            if inst.reference in exclude_refs:
                continue
            chiplet_def = self.get_def(inst.reference)
            if chiplet_def:
                bboxes.append(inst.global_aabb(chiplet_def))
        if not bboxes:
            return AABB(0, 0, 0, 0)
        return AABB(
            min(b.x1 for b in bboxes),
            min(b.y1 for b in bboxes),
            max(b.x2 for b in bboxes),
            max(b.y2 for b in bboxes)
        )

    def get_d2d_ip_positions(self, conn: D2DConnection) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Return ((sx, sy), (tx, ty)) global positions of a D2D connection's IPs."""
        src_inst = self.get_instance(conn.source_inst)
        tgt_inst = self.get_instance(conn.target_inst)
        if not src_inst or not tgt_inst:
            return None
        src_def = self.get_def(src_inst.reference)
        tgt_def = self.get_def(tgt_inst.reference)
        if not src_def or not tgt_def:
            return None
        src_pos = src_inst.global_ip_position(src_def, conn.source_ip)
        tgt_pos = tgt_inst.global_ip_position(tgt_def, conn.target_ip)
        if src_pos is None or tgt_pos is None:
            return None
        return (src_pos, tgt_pos)


@dataclass
class ViolationReport:
    """Report of constraint violations for a placement."""
    hard_violations: List[str] = field(default_factory=list)
    soft_scores: Dict[str, float] = field(default_factory=dict)
    score_details: Dict[str, Dict] = field(default_factory=dict)  # per-rule detail: formula, vars, values
    total_score: float = 0.0

    @property
    def is_valid(self) -> bool:
        return len(self.hard_violations) == 0

    def __repr__(self) -> str:
        lines = ["=== Violation Report ==="]
        if self.hard_violations:
            lines.append(f"Hard Violations ({len(self.hard_violations)}):")
            for v in self.hard_violations:
                lines.append(f"  - {v}")
        else:
            lines.append("Hard Rules: ALL SATISFIED")
        lines.append("Soft Scores:")
        for k, v in self.soft_scores.items():
            lines.append(f"  {k}: {v:.4f}")
        lines.append(f"Total Score: {self.total_score:.4f}")
        return "\n".join(lines)


@dataclass
class PlacementSolution:
    """A complete placement solution with score and violations."""
    design: DesignModel
    instance_poses: Dict[str, InstancePose]  # {inst_name: pose}
    interposer_size: Tuple[float, float] = (0.0, 0.0)
    score: float = 0.0
    report: ViolationReport = field(default_factory=ViolationReport)

    def get_instance_pose(self, inst_name: str) -> Optional[InstancePose]:
        return self.instance_poses.get(inst_name)

    def apply_to_design(self) -> None:
        """Apply this solution's poses back to the design model instances."""
        for inst in self.design.instances:
            if inst.name in self.instance_poses:
                inst.pose = self.instance_poses[inst.name].copy()

    def __repr__(self) -> str:
        return (f"PlacementSolution(score={self.score:.4f}, "
                f"instances={len(self.instance_poses)}, "
                f"valid={self.report.is_valid})")
