"""
Geometry Engine for 3D IC Chiplets Coarse-Placement System.

Handles all coordinate transformations, AABB computations, collision detection,
and geometric operations on chiplet instances.
"""

import math
from typing import Tuple, Optional, List
from .models import AABB, InstancePose, ChipletDef


class GeometryEngine:
    """Centralized geometric computation engine. All methods are static/stateless."""

    # Orientation to rotation angle (degrees, CCW)
    ORIENTATION_ANGLES = {
        "R0": 0,
        "R90": 90,
        "R180": 180,
        "R270": 270,
    }

    # Combined orientations like "MX_R90", "MY_R180"
    @staticmethod
    def parse_orientation(orientation: str) -> Tuple[float, bool, bool]:
        """Parse orientation string into (angle_deg, flip_mx, flip_my).
        
        Handles: R0, R90, R180, R270, MX, MY, MX_R90, MY_R180, etc.
        """
        flip_mx = "MX" in orientation
        flip_my = "MY" in orientation
        
        angle = 0.0
        for key in GeometryEngine.ORIENTATION_ANGLES:
            if key in orientation:
                angle = GeometryEngine.ORIENTATION_ANGLES[key]
                break
        
        return angle, flip_mx, flip_my

    @staticmethod
    def _get_transform(pose: InstancePose) -> Tuple[float, bool, bool]:
        """Get angle and flip flags from a pose (combining orientation string and flip field)."""
        angle, flip_mx, flip_my = GeometryEngine.parse_orientation(pose.orientation)
        # Also read pose.flip field (e.g. "MX", "MY") to override
        if pose.flip == "MX":
            flip_mx = True
        elif pose.flip == "MY":
            flip_my = True
        return angle, flip_mx, flip_my

    @staticmethod
    def _get_bbox_offset(pose: InstancePose, chiplet_width: float, chiplet_height: float) -> Tuple[float, float]:
        """Get the offset to translate the bounding box's left-bottom corner to (0,0).
        
        After flip+rotate around origin, the transformed corners may have negative coordinates.
        This offset shifts the bounding box so its minX/minY corner is at (0,0).
        """
        corners = [(0, 0), (chiplet_width, 0), (0, chiplet_height), (chiplet_width, chiplet_height)]
        transformed = []
        for cx, cy in corners:
            # flip around origin (left-bottom corner)
            fcx, fcy = cx, cy
            if pose.flip == "MX":
                fcy = -fcy
            elif pose.flip == "MY":
                fcx = -fcx
            # rotate around origin (left-bottom corner), CCW
            angle = GeometryEngine.ORIENTATION_ANGLES.get(pose.orientation, 0)
            rad = math.radians(angle)
            cos_a = math.cos(rad)
            sin_a = math.sin(rad)
            rcx = fcx * cos_a - fcy * sin_a
            rcy = fcx * sin_a + fcy * cos_a
            transformed.append((rcx, rcy))
        
        min_x = min(c[0] for c in transformed)
        min_y = min(c[1] for c in transformed)
        return -min_x, -min_y

    @staticmethod
    def local_to_global(pose: InstancePose, local_x: float, local_y: float,
                        chiplet_width: float = 0.0, chiplet_height: float = 0.0) -> Tuple[float, float]:
        """Transform a point from chiplet local coordinates to global coordinates.
        
        Rotation center is the left-bottom corner (0,0) of the chiplet.
        After flip+rotate, the bounding box is translated so its left-bottom corner is at (0,0),
        then translated to the instance position (pose.x, pose.y).
        
        This ensures that after any orientation, the instance's loc = (pose.x, pose.y)
        is always the bounding box's left-bottom corner in global coordinates.
        """
        # 1. Flip around origin (left-bottom corner)
        fx = local_x
        fy = local_y
        if pose.flip == "MX":
            fy = -fy
        elif pose.flip == "MY":
            fx = -fx
        
        # 2. Rotate around origin (left-bottom corner), CCW
        angle = GeometryEngine.ORIENTATION_ANGLES.get(pose.orientation, 0)
        rad = math.radians(angle)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        rx = fx * cos_a - fy * sin_a
        ry = fx * sin_a + fy * cos_a
        
        # 3. Translate bounding box's left-bottom to (0,0)
        dx, dy = GeometryEngine._get_bbox_offset(pose, chiplet_width, chiplet_height)
        
        # 4. Translate to instance position
        return rx + dx + pose.x, ry + dy + pose.y

    @staticmethod
    def global_to_local(pose: InstancePose, global_x: float, global_y: float,
                        chiplet_width: float = 0.0, chiplet_height: float = 0.0) -> Tuple[float, float]:
        """Inverse transform: global -> local coordinates.
        
        Inverse of local_to_global: undo instance translation, undo bbox translation,
        undo rotation, undo flip.
        """
        # 1. Undo instance translation + bbox translation
        dx, dy = GeometryEngine._get_bbox_offset(pose, chiplet_width, chiplet_height)
        tx = global_x - pose.x - dx
        ty = global_y - pose.y - dy
        
        # 2. Undo rotation (rotate by -angle)
        angle = GeometryEngine.ORIENTATION_ANGLES.get(pose.orientation, 0)
        rad = math.radians(-angle)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        rx = tx * cos_a - ty * sin_a
        ry = tx * sin_a + ty * cos_a
        
        # 3. Undo flip (flip is its own inverse)
        if pose.flip == "MX":
            ry = -ry
        elif pose.flip == "MY":
            rx = -rx
        
        return rx, ry

    @staticmethod
    def compute_global_aabb(pose: InstancePose, chiplet_def: ChipletDef) -> AABB:
        """Compute the AABB of a chiplet instance in global coordinates."""
        # The four corners of the chiplet in local coordinates
        w, h = chiplet_def.width, chiplet_def.height
        corners = [
            (0.0, 0.0), (w, 0.0), (w, h), (0.0, h)
        ]
        
        global_corners = [GeometryEngine.local_to_global(pose, lx, ly, w, h) for lx, ly in corners]
        
        xs = [p[0] for p in global_corners]
        ys = [p[1] for p in global_corners]
        
        return AABB(min(xs), min(ys), max(xs), max(ys))

    @staticmethod
    def compute_ip_global_position(pose: InstancePose, chiplet_def: ChipletDef, 
                                   ip_name: str) -> Optional[Tuple[float, float]]:
        """Compute the global center position of an IP by its name."""
        # Find the IP entry in omap
        for entry in chiplet_def.omap_entries:
            if entry.name == ip_name:
                obj_size = chiplet_def.get_object_size(entry.obj_type)
                # IP center in local coords
                local_cx = entry.loc_x + obj_size[0] / 2.0
                local_cy = entry.loc_y + obj_size[1] / 2.0
                return GeometryEngine.local_to_global(pose, local_cx, local_cy, 
                                                        chiplet_def.width, chiplet_def.height)
        return None

    @staticmethod
    def compute_ip_global_aabb(pose: InstancePose, chiplet_def: ChipletDef,
                               ip_name: str) -> Optional[AABB]:
        """Compute the global AABB of an IP by its name."""
        for entry in chiplet_def.omap_entries:
            if entry.name == ip_name:
                obj_size = chiplet_def.get_object_size(entry.obj_type)
                # Four corners of IP in local coords
                local_corners = [
                    (entry.loc_x, entry.loc_y),
                    (entry.loc_x + obj_size[0], entry.loc_y),
                    (entry.loc_x + obj_size[0], entry.loc_y + obj_size[1]),
                    (entry.loc_x, entry.loc_y + obj_size[1]),
                ]
                global_corners = [GeometryEngine.local_to_global(pose, lx, ly,
                                                                  chiplet_def.width, chiplet_def.height) 
                                   for lx, ly in local_corners]
                xs = [p[0] for p in global_corners]
                ys = [p[1] for p in global_corners]
                return AABB(min(xs), min(ys), max(xs), max(ys))
        return None

    @staticmethod
    def check_overlap(aabb1: AABB, aabb2: AABB, margin: float = 0.0) -> bool:
        """Check if two AABBs overlap (with optional margin)."""
        return aabb1.overlaps(aabb2, margin)

    @staticmethod
    def check_abutment(aabb1: AABB, aabb2: AABB, tol: float = 1.0) -> bool:
        """Check if two AABBs are abutting (touching with zero gap, within tolerance).
        
        Returns True if they share an edge with no gap and no overlap.
        """
        # X-direction abutment
        x_abut = (abs(aabb1.x2 - aabb2.x1) < tol or abs(aabb2.x2 - aabb1.x1) < tol)
        # Y-direction overlap must exist for X-abutment
        y_overlap = not (aabb1.y2 <= aabb2.y1 or aabb2.y2 <= aabb1.y1)
        
        # Y-direction abutment
        y_abut = (abs(aabb1.y2 - aabb2.y1) < tol or abs(aabb2.y2 - aabb1.y1) < tol)
        # X-direction overlap must exist for Y-abutment
        x_overlap = not (aabb1.x2 <= aabb2.x1 or aabb2.x2 <= aabb1.x1)
        
        return (x_abut and y_overlap) or (y_abut and x_overlap)

    @staticmethod
    def check_containment(outer: AABB, inner: AABB, margin: float = 0.0) -> bool:
        """Check if inner is fully contained within outer."""
        return outer.contains(inner.inflate(margin))

    @staticmethod
    def compute_mbr(aabbs: List[AABB]) -> AABB:
        """Compute Minimum Bounding Rectangle of a list of AABBs."""
        if not aabbs:
            return AABB(0, 0, 0, 0)
        return AABB(
            min(b.x1 for b in aabbs),
            min(b.y1 for b in aabbs),
            max(b.x2 for b in aabbs),
            max(b.y2 for b in aabbs)
        )

    @staticmethod
    def slide_to_contact(mover_aabb: AABB, target_aabb: AABB, direction: str) -> float:
        """Compute how much to slide 'mover' in 'direction' until it touches 'target'.
        
        direction: 'left', 'right', 'up', 'down'
        Returns the new x or y coordinate (depending on direction).
        """
        if direction == 'left':
            return target_aabb.x2
        elif direction == 'right':
            return target_aabb.x1 - mover_aabb.width
        elif direction == 'down':
            return target_aabb.y2
        elif direction == 'up':
            return target_aabb.y1 - mover_aabb.height
        return 0.0

    @staticmethod
    def resolve_overlap(aabb1: AABB, aabb2: AABB) -> Tuple[float, float, float, float]:
        """Compute the minimal displacement to resolve overlap between two AABBs.
        
        Returns (dx1, dy1, dx2, dy2) where (dx1, dy1) is the displacement for aabb1
        and (dx2, dy2) is the displacement for aabb2. Each pair pushes apart by half
        the overlap distance in each axis.
        """
        if not aabb1.overlaps(aabb2):
            return 0.0, 0.0, 0.0, 0.0
        
        # Overlap amounts
        x_overlap = min(aabb1.x2, aabb2.x2) - max(aabb1.x1, aabb2.x1)
        y_overlap = min(aabb1.y2, aabb2.y2) - max(aabb1.y1, aabb2.y1)
        
        dx, dy = 0.0, 0.0
        if x_overlap < y_overlap:
            # Push in X direction (smaller overlap = preferred resolution direction)
            if aabb1.center[0] < aabb2.center[0]:
                dx = -x_overlap / 2.0
            else:
                dx = x_overlap / 2.0
        else:
            # Push in Y direction
            if aabb1.center[1] < aabb2.center[1]:
                dy = -y_overlap / 2.0
            else:
                dy = y_overlap / 2.0
        
        return dx, dy, -dx, -dy

    @staticmethod
    def manhattan_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    @staticmethod
    def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
