"""
3Dblox format parser.

Handles .3dbx, .3dbv, .3dbo, .omap, and .connection files.
Parses YAML-like 3Dblox format into DesignModel.
"""

import os

try:
    import yaml
except ImportError:
    from . import simple_yaml as yaml

from typing import Optional, Dict, Any, List
from .models import (
    DesignModel, ChipletDef, ChipletInst, ObjectDef, ObjectMapEntry,
    InstancePose, Flexibility, D2DConnection
)


class ParserError(Exception):
    """Raised when parsing fails."""
    pass


class Parser:
    """Parse 3Dblox files into a DesignModel."""

    def __init__(self, base_dir: str = ""):
        self.base_dir = base_dir

    def parse_design_from_content(self, dbx_content: str, base_dir: str = "") -> DesignModel:
        """Parse a design from .3dbx content string (for browser-uploaded files).
        
        Saves content to a temp file, then parses normally.
        Reference files (.3dbv, .3dbo, .omap) are resolved relative to base_dir.
        """
        import tempfile
        
        # Save content to temp file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.3dbx', delete=False, encoding='utf-8') as f:
            f.write(dbx_content)
            temp_path = f.name
        
        try:
            return self.parse_design(temp_path, base_dir=base_dir or self.base_dir)
        finally:
            os.unlink(temp_path)
    
    def parse_design(self, dbx_path: str, base_dir: str = None) -> DesignModel:
        """Parse a full design from a .3dbx file, including all referenced files."""
        if base_dir is not None:
            self.base_dir = base_dir
        else:
            self.base_dir = os.path.dirname(os.path.abspath(dbx_path))
        design = DesignModel()
        design.raw_data = {}

        # 1. Parse the top-level .3dbx file
        dbx_data = self._load_yaml(dbx_path)
        design.name = dbx_data.get("Design", {}).get("name", "")
        design.raw_data["3dbx"] = dbx_data

        # 2. Parse .3dbv includes from the .3dbx file
        dbv_includes = dbx_data.get("Header", {}).get("include", [])
        for dbv_file in dbv_includes:
            dbv_path = self._resolve_path(dbv_file)
            if dbv_path and os.path.exists(dbv_path):
                self._parse_3dbv(dbv_path, design)

        # 3. Parse ChipletInst from .3dbx
        chiplet_inst = dbx_data.get("ChipletInst", {})
        for inst_name, inst_data in chiplet_inst.items():
            pose = InstancePose(x=0, y=0, z=0, orientation="R0", flip="None")
            inst = ChipletInst(
                name=inst_name,
                reference=inst_data.get("reference", ""),
                is_master=inst_data.get("is_master", True),
                pose=pose,
                flexibility=Flexibility()
            )
            design.instances.append(inst)

        # 4. Parse Stack (positions and orientations)
        stack = dbx_data.get("Stack", {})
        for inst_name, stack_data in stack.items():
            inst = design.get_instance(inst_name)
            if inst:
                loc = stack_data.get("loc", [0, 0, 0])
                inst.pose.x = float(loc[0]) if len(loc) > 0 else 0.0
                inst.pose.y = float(loc[1]) if len(loc) > 1 else 0.0
                # z can be in loc[2] or as a separate "z" field
                if len(loc) > 2:
                    inst.pose.z = float(loc[2])
                else:
                    inst.pose.z = float(stack_data.get("z", 0.0))

                # Parse composite orient: flip parts + rotation part
                # Examples: "R0" → flip=None, rotation=R0
                #           "MX_R90" → flip=MX, rotation=R90
                #           "MX_MZ_R0" → flip=MX, mz=True, rotation=R0
                orient_str = stack_data.get("orient", "R0")
                parts = orient_str.split("_")
                # Rotation parts are R0, R90, R180, R270
                rotation_parts = [p for p in parts if p in {"R0", "R90", "R180", "R270"}]
                flip_parts = [p for p in parts if p not in {"R0", "R90", "R180", "R270"}]

                if rotation_parts:
                    inst.pose.orientation = rotation_parts[-1]
                else:
                    inst.pose.orientation = "R0"

                # Extract MZ (Z-axis flip) from flip_parts
                if "MZ" in flip_parts:
                    inst.pose.mz = True
                    flip_parts = [p for p in flip_parts if p != "MZ"]
                else:
                    inst.pose.mz = False

                if flip_parts:
                    inst.pose.flip = "_".join(flip_parts)
                else:
                    inst.pose.flip = "None"

        # 5. Parse D2D connections if provided externally (not auto-loaded from directory)
        # Connections must be uploaded separately via API after design load
        
        # 6. Set interposer reference
        if "Interposer" in design.chiplet_defs:
            design.interposer = design.chiplet_defs["Interposer"]
        
        return design
    
    def parse_connections(self, content: str) -> List[D2DConnection]:
        """Parse D2D connection content from a string (not from file)."""
        return self._parse_connection_content(content)
    
    def _parse_connection_content(self, content: str) -> List[D2DConnection]:
        """Parse connection file content directly from string."""
        connections = []
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) >= 2:
                source_full = parts[0]  # e.g., "u_SOC_0.hbm_0"
                target_full = parts[1]  # e.g., "u_HBM_0.PHY0"
                lsi_inst = parts[2] if len(parts) > 2 else None
                
                source_parts = source_full.split('.')
                target_parts = target_full.split('.')
                
                if len(source_parts) == 2 and len(target_parts) == 2:
                    conn = D2DConnection(
                        source_inst=source_parts[0],
                        source_ip=source_parts[1],
                        target_inst=target_parts[0],
                        target_ip=target_parts[1],
                        lsi_inst=lsi_inst or "",
                        is_external="~" in line
                    )
                    connections.append(conn)
        return connections

    def _resolve_path(self, filepath: str) -> Optional[str]:
        """Resolve a relative path to an absolute path."""
        if not filepath:
            return None
        # Remove leading ./ or .\ 
        if filepath.startswith("./") or filepath.startswith(".\\"):
            filepath = filepath[2:]
        elif filepath.startswith(".") and len(filepath) > 1 and filepath[1] in "/\\":
            filepath = filepath[2:]
        
        abs_path = os.path.join(self.base_dir, filepath)
        if os.path.exists(abs_path):
            return abs_path
        return None

    def _load_yaml(self, path: str) -> Dict[str, Any]:
        """Load a YAML file, replacing tabs with spaces."""
        with open(path, "r", encoding="utf-8") as f:
            content = f.read().replace("\t", "  ")
        return yaml.safe_load(content) or {}

    def _parse_3dbv(self, dbv_path: str, design: DesignModel) -> None:
        """Parse a .3dbv file (ChipletDef) and its referenced .3dbo and .omap files."""
        dbv_data = self._load_yaml(dbv_path)
        design.raw_data[os.path.basename(dbv_path)] = dbv_data

        # Parse nested includes in this .3dbv
        includes = dbv_data.get("Header", {}).get("include", [])
        for inc in includes:
            inc_path = self._resolve_path(inc)
            if inc_path and os.path.exists(inc_path):
                self._parse_3dbv(inc_path, design)

        # Parse ChipletDef entries
        chiplet_defs = dbv_data.get("ChipletDef", {})
        for name, def_data in chiplet_defs.items():
            chiplet_def = self._build_chiplet_def(name, def_data)
            design.chiplet_defs[name] = chiplet_def

            # Auto-load .3dbo and .omap
            self._load_3dbo(chiplet_def, def_data)
            self._load_omap(chiplet_def, def_data)

    def _build_chiplet_def(self, name: str, def_data: Dict[str, Any]) -> ChipletDef:
        """Build a ChipletDef from parsed YAML data."""
        size = def_data.get("size", [0, 0])
        if not isinstance(size, (list, tuple)) or len(size) < 2:
            size = [0, 0]
        
        seal_ring = def_data.get("seal_ring", [0, 0, 0, 0])
        if isinstance(seal_ring, (int, float)):
            seal_ring = [seal_ring] * 4
        
        scribe_line = def_data.get("scribe_line_remaining_width", [0, 0, 0, 0])
        if isinstance(scribe_line, (int, float)):
            scribe_line = [scribe_line] * 4

        omap_file = def_data.get("omap", "")
        if isinstance(omap_file, list) and omap_file:
            omap_file = omap_file[0]

        external = def_data.get("external", {})
        dbo_file = None
        if isinstance(external, dict):
            dbo_raw = external.get("3dbo_file", "")
            if isinstance(dbo_raw, list) and dbo_raw:
                dbo_raw = dbo_raw[0]
            if isinstance(dbo_raw, str):
                dbo_file = dbo_raw.strip("[]")

        return ChipletDef(
            name=name,
            size=(float(size[0]), float(size[1])),
            shrink=float(def_data.get("shrink", 1.0)),
            thickness=float(def_data.get("thickness", 0.0)),
            seal_ring=[float(v) for v in seal_ring] if len(seal_ring) == 4 else [0.0]*4,
            scribe_line=[float(v) for v in scribe_line] if len(scribe_line) == 4 else [0.0]*4,
            omap_file=str(omap_file) if omap_file else None,
            dbo_file=dbo_file
        )

    def _load_3dbo(self, chiplet_def: ChipletDef, def_data: Dict[str, Any]) -> None:
        """Load .3dbo file referenced by a ChipletDef."""
        dbo_file = chiplet_def.dbo_file
        if not dbo_file:
            # Try auto-detect
            dbo_file = f"{chiplet_def.name}.3dbo"
        
        dbo_path = self._resolve_path(dbo_file)
        if dbo_path and os.path.exists(dbo_path):
            dbo_data = self._load_yaml(dbo_path)
            obj_defs = dbo_data.get("ObjectDef", {})
            for obj_name, obj_info in obj_defs.items():
                size = obj_info.get("size", [0, 0])
                layer = obj_info.get("layer", [])
                chiplet_def.object_defs[obj_name] = ObjectDef(
                    name=obj_name,
                    size=(float(size[0]) if len(size) > 0 else 0.0,
                          float(size[1]) if len(size) > 1 else 0.0),
                    layer=layer if isinstance(layer, list) else []
                )

    def _load_omap(self, chiplet_def: ChipletDef, def_data: Dict[str, Any]) -> None:
        """Load .omap file referenced by a ChipletDef."""
        omap_file = chiplet_def.omap_file
        if not omap_file:
            omap_file = f"{chiplet_def.name}.omap"
        
        omap_path = self._resolve_path(omap_file)
        if omap_path and os.path.exists(omap_path):
            with open(omap_path, "r", encoding="utf-8") as f:
                lines = f.readlines()
            
            start_idx = 0
            if lines and ("type" in lines[0].lower() or "name" in lines[0].lower()):
                start_idx = 1
            
            for line in lines[start_idx:]:
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) >= 5:
                    chiplet_def.omap_entries.append(ObjectMapEntry(
                        obj_type=parts[0],
                        name=parts[1],
                        loc_x=float(parts[2]),
                        loc_y=float(parts[3]),
                        orientation=parts[4]
                    ))

