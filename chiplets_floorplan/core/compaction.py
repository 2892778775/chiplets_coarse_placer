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
        Compute the origin (x, y) of the Interposer such that all chiplets
        are enclosed with the specified margin.
        Returns (x, y).
        """
        mbr = self.design.mbr_of_instances()
        
        x = mbr.x1 - self.min_enclosure
        y = mbr.y1 - self.min_enclosure
        
        return (x, y)

    def update_interposer(self) -> None:
        """
        Update the Interposer chiplet definition in the design model
        with the computed size and origin.
        """
        if not self.design.interposer:
            return
        
        width, height = self.compute_interposer_size()
        origin_x, origin_y = self.compute_interposer_origin()
        
        self.design.interposer.size = (width, height)
        
        # Update Interposer instance position if it exists
        for inst in self.design.instances:
            if inst.reference == "Interposer":
                inst.pose.x = origin_x
                inst.pose.y = origin_y
                break
