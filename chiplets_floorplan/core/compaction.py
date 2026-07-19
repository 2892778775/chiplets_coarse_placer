"""
Compaction: determine minimum Interposer size after all chiplets are placed.

After all real chiplets are placed, compute the Minimum Bounding Rectangle (MBR)
and add user-specified minimum enclosure on all sides.

User API:
- min_enclosure: minimum distance from outermost chiplet edge to Interposer edge.
"""

from .models import DesignModel, AABB


class Compaction:
    """Compute the compacted Interposer size with user-specified enclosure."""

    def __init__(self, design: DesignModel, min_enclosure: float = 500.0):
        self.design = design
        self.min_enclosure = min_enclosure

    def compute_interposer_size(self) -> tuple:
        """
        Compute the minimum Interposer size that contains all chiplets
        with the specified enclosure margin.
        Returns (width, height).
        """
        mbr = self.design.mbr_of_instances()
        
        if mbr.width == 0 and mbr.height == 0:
            return (0, 0)
        
        width = mbr.width + 2 * self.min_enclosure
        height = mbr.height + 2 * self.min_enclosure
        
        return (width, height)

    def compute_interposer_origin(self) -> tuple:
        """
        Compute the origin (x, y) of the base layer such that it is centered
        on the chiplet MBR. For a resizable Interposer this is equivalent to
        (mbr.x1 - enclosure, mbr.y1 - enclosure); for a fixed-size base (RW)
        centering keeps hard rule H6 satisfied.
        """
        mbr = self.design.mbr_of_instances()
        width, height = self.compute_interposer_size()

        # If the base def exists and will NOT be resized (fixed base like RW),
        # center it on the MBR using its own size.
        if not self.design.interposer:
            base_ref = self.design.reference_def_name()
            base_def = self.design.get_def(base_ref) if base_ref else None
            if base_def:
                width, height = base_def.width, base_def.height

        mbr_cx = (mbr.x1 + mbr.x2) / 2.0
        mbr_cy = (mbr.y1 + mbr.y2) / 2.0
        return (mbr_cx - width / 2.0, mbr_cy - height / 2.0)

    def update_interposer(self) -> None:
        """
        Update the base layer (Interposer / RW) with the computed size and origin.

        When an explicit Interposer def exists, its size is updated to the
        compacted size. For CoW designs with a fixed base def (e.g. RW), only
        the base instance is repositioned so it encloses the placement with
        the enclosure margin; the def size is left untouched.
        """
        origin_x, origin_y = self.compute_interposer_origin()

        if self.design.interposer:
            width, height = self.compute_interposer_size()
            self.design.interposer.size = (width, height)

        # Update base-layer instance position if it exists
        for inst in self.design.instances:
            if self.design.is_base_instance(inst):
                inst.pose.x = origin_x
                inst.pose.y = origin_y
                break
