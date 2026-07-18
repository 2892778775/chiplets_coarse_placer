"""
Dummy Die Filler.

After all real chiplets are placed, fill remaining empty regions with Dummy Dies.

User-configurable parameters (API):
- min_area_threshold: minimum area (um^2) for a region to be filled. Default: 1000*1000 = 1,000,000
- aspect_ratio_range: (min_ar, max_ar) for dummy die aspect ratio. Default: (1.0, 2.0)
  where AR = max(width, height) / min(width, height)
"""

import math
from typing import List, Tuple, Optional
from .models import DesignModel, ChipletInst, ChipletDef, InstancePose, AABB, Flexibility
from .geometry import GeometryEngine


class DummyFiller:
    """Fill empty regions with Dummy Dies."""

    def __init__(self, design: DesignModel,
                 min_area_threshold: float = 1_000_000,  # 1000 * 1000
                 aspect_ratio_range: Tuple[float, float] = (1.0, 2.0)):
        self.design = design
        self.min_area_threshold = min_area_threshold
        self.min_ar = aspect_ratio_range[0]
        self.max_ar = aspect_ratio_range[1]

    def fill(self, mbr: AABB) -> List[ChipletInst]:
        """
        Fill empty regions inside the MBR with Dummy Dies.
        Returns a list of Dummy Die instances.
        """
        dummy_instances = []

        # Get all real chiplet bboxes. The base-layer instance (Interposer /
        # RW) is excluded: dummy dies sit *inside* the base footprint, so the
        # base itself must not count as occupied space.
        real_bboxes = []
        for inst in self.design.instances:
            if self.design.is_base_instance(inst):
                continue
            chiplet_def = self.design.get_def(inst.reference)
            if chiplet_def and not inst.reference.startswith("Dummy"):
                real_bboxes.append(inst.global_aabb(chiplet_def))

        # Restrict the fill region to the base-layer footprint so dummy dies
        # never extend beyond the base and break the enclosure rule (H6).
        base_inst = self.design.base_instance()
        if base_inst:
            base_def = self.design.get_def(base_inst.reference)
            if base_def:
                b = base_inst.global_aabb(base_def)
                x1, y1 = max(mbr.x1, b.x1), max(mbr.y1, b.y1)
                x2, y2 = min(mbr.x2, b.x2), min(mbr.y2, b.y2)
                if x2 > x1 and y2 > y1:
                    mbr = AABB(x1, y1, x2, y2)

        # Seed the dummy index from existing dummies. Instances created here
        # are only returned to the caller (not appended to the design), so a
        # local counter is required to keep names/defs unique within this run.
        self._next_dummy_idx = len([d for d in self.design.instances
                                    if d.reference.startswith("Dummy")]) + 1

        # Identify empty rectangular regions
        empty_regions = self._identify_empty_regions(mbr, real_bboxes)
        
        # Group regions by size and generate dummy dies
        for region in empty_regions:
            width = region.x2 - region.x1
            height = region.y2 - region.y1
            area = width * height
            
            if area < self.min_area_threshold:
                continue
            
            # Check aspect ratio
            ar = max(width, height) / min(width, height) if min(width, height) > 0 else 1.0
            if ar < self.min_ar or ar > self.max_ar:
                # Try to subdivide into smaller rectangles with valid AR
                sub_regions = self._subdivide_region(region)
                for sub in sub_regions:
                    dummy = self._create_dummy_die(sub)
                    if dummy:
                        dummy_instances.append(dummy)
            else:
                dummy = self._create_dummy_die(region)
                if dummy:
                    dummy_instances.append(dummy)

        return dummy_instances

    def _identify_empty_regions(self, mbr: AABB, occupied: List[AABB]) -> List[AABB]:
        """
        Identify all axis-aligned rectangular empty regions within the MBR.
        Uses a sweep-line approach: create a grid from all x and y coordinates,
        then check each cell.
        """
        x_coords = {mbr.x1, mbr.x2}
        y_coords = {mbr.y1, mbr.y2}
        
        for bbox in occupied:
            x_coords.add(bbox.x1)
            x_coords.add(bbox.x2)
            y_coords.add(bbox.y1)
            y_coords.add(bbox.y2)
        
        sorted_x = sorted(x_coords)
        sorted_y = sorted(y_coords)
        
        regions = []
        for i in range(len(sorted_x) - 1):
            for j in range(len(sorted_y) - 1):
                x1, y1 = sorted_x[i], sorted_y[j]
                x2, y2 = sorted_x[i + 1], sorted_y[j + 1]
                
                if x2 <= x1 or y2 <= y1:
                    continue
                
                candidate = AABB(x1, y1, x2, y2)
                
                # Check if this cell overlaps with any occupied bbox
                overlap = False
                for bbox in occupied:
                    if candidate.overlaps(bbox):
                        overlap = True
                        break
                
                if not overlap:
                    regions.append(candidate)
        
        return regions

    def _subdivide_region(self, region: AABB) -> List[AABB]:
        """
        Subdivide a region into smaller rectangles that meet aspect ratio constraints.
        Uses an iterative tiling approach to avoid infinite recursion.
        """
        width = region.x2 - region.x1
        height = region.y2 - region.y1
        area = width * height
        
        if area < self.min_area_threshold or min(width, height) <= 0:
            return []
        
        ar = max(width, height) / min(width, height) if min(width, height) > 0 else 1.0
        if self.min_ar <= ar <= self.max_ar:
            return [region]
        
        # Tile the region into a grid of valid rectangles
        sub_regions = []
        
        if width > height:
            # Horizontal tiling: each tile has width = height * max_ar (or less for last tile)
            tile_w = height * self.max_ar
            n_tiles = max(1, int(width / tile_w))
            # Ensure at least 2 tiles if AR is too large
            if n_tiles < 2 and ar > self.max_ar:
                n_tiles = 2
            actual_tile_w = width / n_tiles
            for i in range(n_tiles):
                x1 = region.x1 + i * actual_tile_w
                x2 = region.x1 + (i + 1) * actual_tile_w if i < n_tiles - 1 else region.x2
                sub = AABB(x1, region.y1, x2, region.y2)
                sub_w = x2 - x1
                sub_area = sub_w * height
                sub_ar = max(sub_w, height) / min(sub_w, height) if min(sub_w, height) > 0 else 1.0
                if sub_area >= self.min_area_threshold and self.min_ar <= sub_ar <= self.max_ar:
                    sub_regions.append(sub)
        else:
            # Vertical tiling: each tile has height = width * max_ar
            tile_h = width * self.max_ar
            n_tiles = max(1, int(height / tile_h))
            if n_tiles < 2 and ar > self.max_ar:
                n_tiles = 2
            actual_tile_h = height / n_tiles
            for i in range(n_tiles):
                y1 = region.y1 + i * actual_tile_h
                y2 = region.y1 + (i + 1) * actual_tile_h if i < n_tiles - 1 else region.y2
                sub = AABB(region.x1, y1, region.x2, y2)
                sub_h = y2 - y1
                sub_area = width * sub_h
                sub_ar = max(width, sub_h) / min(width, sub_h) if min(width, sub_h) > 0 else 1.0
                if sub_area >= self.min_area_threshold and self.min_ar <= sub_ar <= self.max_ar:
                    sub_regions.append(sub)
        
        return sub_regions

    # Hairline clearance inset on every dummy die. Adjacent empty-region cells
    # share edges exactly, so without an inset neighboring dummies touch at
    # x2 == x1; export/re-parse float rounding (4-decimal coordinates) can
    # then turn the touching pair into a ~1e-12 overlap and trip H1.
    DUMMY_EDGE_INSET = 1e-3  # um; far above float noise, far below real sizes

    def _create_dummy_die(self, region: AABB) -> Optional[ChipletInst]:
        """Create a Dummy Die instance for a given region."""
        width = (region.x2 - region.x1) - 2 * self.DUMMY_EDGE_INSET
        height = (region.y2 - region.y1) - 2 * self.DUMMY_EDGE_INSET
        area = width * height

        if area < self.min_area_threshold or width <= 0 or height <= 0:
            return None
        
        # Generate a unique dummy name (local counter seeded in fill()).
        dummy_name = f"Dummy_{self._next_dummy_idx}"
        self._next_dummy_idx += 1
        
        # Create chiplet def if not exists
        if dummy_name not in self.design.chiplet_defs:
            self.design.chiplet_defs[dummy_name] = ChipletDef(
                name=dummy_name,
                size=(width, height),
                shrink=1.0,
                thickness=100,
                seal_ring=[0, 0, 0, 0],
                scribe_line=[0, 0, 0, 0]
            )
        
        instance_name = f"u_{dummy_name}_1"
        dummy_inst = ChipletInst(
            name=instance_name,
            reference=dummy_name,
            is_master=True,
            pose=InstancePose(x=region.x1 + self.DUMMY_EDGE_INSET,
                              y=region.y1 + self.DUMMY_EDGE_INSET, z=0),
            flexibility=Flexibility(status="fixed", shift=False, rotate=False, flip=False, resize=False)
        )
        return dummy_inst
