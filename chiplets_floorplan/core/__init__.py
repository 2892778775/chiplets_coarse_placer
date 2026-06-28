"""Core module exports."""
from .models import (
    AABB, ChipletDef, ChipletInst, DesignModel, D2DConnection,
    InstancePose, ObjectDef, ObjectMapEntry, PlacementSolution, ViolationReport
)
from .geometry import GeometryEngine

# Lazy imports for heavy dependencies
# from .parser import Parser
# from .constraints import ConstraintChecker
# from .placer import Placer
# from .d2d_router import D2DRouter
# from .dummy_filler import DummyFiller
# from .compaction import Compaction
# from .exporter import Exporter

__all__ = [
    "AABB", "ChipletDef", "ChipletInst", "DesignModel", "D2DConnection",
    "InstancePose", "ObjectDef", "ObjectMapEntry", "PlacementSolution", "ViolationReport",
    "GeometryEngine",
]