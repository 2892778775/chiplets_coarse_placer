"""
Flask backend for the 3D IC Chiplets Coarse-Placement System (v2).

Provides REST APIs for:
- Loading 3Dblox designs
- Running coarse-placement
- D2D refinement, LSI generation
- Exporting optimized 3Dblox files
- Real-time score evaluation
"""

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import os
import sys
import json
import traceback
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from chiplets_floorplan.core.parser import Parser
from chiplets_floorplan.core.placer import Placer
from chiplets_floorplan.core.d2d_router import D2DRouter
from chiplets_floorplan.core.compaction import Compaction
from chiplets_floorplan.core.exporter import Exporter
from chiplets_floorplan.core.constraints import ConstraintChecker
from chiplets_floorplan.core.geometry import GeometryEngine
from chiplets_floorplan.core.models import AABB, InstancePose, Flexibility, D2DConnection

app = Flask(__name__, static_folder='static', template_folder='templates')
CORS(app)

app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
app.config['TEMPLATES_AUTO_RELOAD'] = True

@app.after_request
def add_cache_control(response):
    """Disable browser caching for all responses to ensure latest JS/CSS is loaded."""
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response

UPLOAD_FOLDER = os.path.join(tempfile.gettempdir(), 'chiplets_floorplan_uploads')
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# In-memory session state (per-server, no persistence)
class SessionState:
    def __init__(self):
        self.design = None
        self.solution = None
        self.dbx_path = ""
        self.reference_instance = ""  # User-selected reference chiplet instance
        self.config = {
            "enclosure": 500.0,
            "sa_iterations": 5000,
        }

state = SessionState()


def _find_base_dir_for_includes(dbx_content: str) -> str:
    """Search filesystem for a directory that contains the .3dbv files
    referenced by the .3dbx content. Returns the best-matching directory
    or empty string."""
    try:
        import yaml
    except ImportError:
        from chiplets_floorplan.core import simple_yaml as yaml

    try:
        data = yaml.safe_load(dbx_content) or {}
    except Exception:
        return ""

    includes = data.get("Header", {}).get("include", [])
    if not includes:
        return ""

    # Collect candidate directories
    candidates = {os.getcwd()}
    # Add immediate subdirectories of cwd and project root
    for root in [os.getcwd(), os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))]:
        if not os.path.isdir(root):
            continue
        for entry in os.listdir(root):
            d = os.path.join(root, entry)
            if os.path.isdir(d):
                candidates.add(d)

    best_dir = ""
    best_score = -1

    for d in candidates:
        score = 0
        for inc in includes:
            inc_clean = inc.lstrip("./").lstrip(".\\")
            if os.path.exists(os.path.join(d, inc_clean)):
                score += 1
        if score > best_score:
            best_score = score
            best_dir = d

    return best_dir if best_score > 0 else ""


@app.after_request
def add_no_cache_headers(response):
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = 'Thu, 01 Jan 1970 00:00:00 GMT'
    return response

# ------------------------------------------------------------------
# Frontend route
# ------------------------------------------------------------------

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

# ------------------------------------------------------------------
# Design loading
# ------------------------------------------------------------------

@app.route('/api/load_design', methods=['POST'])
def load_design():
    try:
        data = request.get_json()
        dbx_path = data.get('dbx_path', '')
        connection_content = data.get('connection_content', '')
        
        if not dbx_path or not os.path.exists(dbx_path):
            return jsonify({'success': False, 'error': 'Invalid or missing .3dbx file path'})
        
        parser = Parser()
        design = parser.parse_design(dbx_path)
        
        # Parse D2D connections if provided by user upload
        if connection_content:
            design.d2d_connections = parser.parse_connections(connection_content)
        
        state.design = design
        state.dbx_path = dbx_path
        state.solution = None
        
        # Default reference: Interposer if exists, otherwise empty
        state.reference_instance = ""
        for inst in design.instances:
            if inst.reference == "Interposer":
                state.reference_instance = inst.name
                break
        
        chiplet_names = [inst.name for inst in design.instances]
        chiplet_types = sorted(set(design.chiplet_defs.keys()))
        
        return jsonify({
            'success': True,
            'design_name': design.name,
            'chiplet_count': len(design.chiplet_defs),
            'instance_count': len(design.instances),
            'd2d_connections': len(design.d2d_connections),
            'chiplet_names': chiplet_names,
            'chiplet_types': chiplet_types
        })
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Get current design data (for frontend rendering)
# ------------------------------------------------------------------

@app.route('/api/get_data', methods=['GET'])
def get_data():
    try:
        if not state.design:
            return jsonify({'success': False, 'error': 'No design loaded'})
        
        design = state.design
        
        # Chiplet definitions
        chiplet_defs = []
        for name, cdef in design.chiplet_defs.items():
            chiplet_defs.append({
                'name': name,
                'size': [cdef.width, cdef.height],
                'thickness': cdef.thickness,
                'shrink': cdef.shrink,
                'seal_ring': cdef.seal_ring,
                'scribe_line': cdef.scribe_line
            })
        
        # Instances with IP positions
        instances = []
        for inst in design.instances:
            inst_data = {
                'name': inst.name,
                'module': inst.reference,
                'is_master': inst.is_master,
                'x': inst.pose.x,
                'y': inst.pose.y,
                'z': inst.pose.z,
                'orientation': inst.pose.orientation,
                'flip': inst.pose.flip,
                'mz': inst.pose.mz,
                'visible': inst.visible,
                'group': inst.group
            }
            # Add IP positions for this instance
            chiplet_def = design.get_def(inst.reference)
            if chiplet_def:
                ips = []
                for entry in chiplet_def.omap_entries:
                    obj_size = chiplet_def.get_object_size(entry.obj_type)
                    local_cx = entry.loc_x + obj_size[0] / 2.0
                    local_cy = entry.loc_y + obj_size[1] / 2.0
                    gx, gy = GeometryEngine.local_to_global(
                        inst.pose, local_cx, local_cy, chiplet_def.width, chiplet_def.height
                    )
                    ips.append({
                        'name': entry.name,
                        'type': entry.obj_type,
                        'local_x': entry.loc_x,
                        'local_y': entry.loc_y,
                        'size': obj_size,
                        'global_x': gx,
                        'global_y': gy
                    })
                inst_data['ips'] = ips
            else:
                inst_data['ips'] = []
            instances.append(inst_data)
        
        # Auto-assign default Z values if all non-Interposer instances have z=0
        non_interposer = [i for i in instances if i['module'] != 'Interposer']
        if non_interposer and all(i['z'] == 0 for i in non_interposer):
            for i in instances:
                if i['module'] == 'Interposer':
                    continue
                elif i['is_master']:
                    i['z'] = 0
                elif 'LSI' in i['module'].upper():
                    i['z'] = 250
                else:
                    i['z'] = 525
            # Also update backend memory for consistency
            for inst in design.instances:
                if inst.reference == 'Interposer':
                    continue
                elif inst.is_master:
                    inst.pose.z = 0
                elif 'LSI' in inst.reference.upper():
                    inst.pose.z = 250
                else:
                    inst.pose.z = 525
        
        # D2D connections with global positions
        connections = []
        for conn in design.d2d_connections:
            positions = design.get_d2d_ip_positions(conn)
            if positions:
                (sx, sy), (tx, ty) = positions
                connections.append({
                    'source_inst': conn.source_inst,
                    'source_ip': conn.source_ip,
                    'target_inst': conn.target_inst,
                    'target_ip': conn.target_ip,
                    'source_x': sx, 'source_y': sy,
                    'target_x': tx, 'target_y': ty
                })
        
        # Score if available
        score_data = None
        if state.solution:
            checker = ConstraintChecker(state.design)
            score_data = {
                'total': state.solution.score,
                'valid': state.solution.report.is_valid,
                'soft_scores': state.solution.report.soft_scores,
                'hard_violations': state.solution.report.hard_violations,
                'score_details': state.solution.report.score_details,
                'weights': checker.weights
            }
        
        return jsonify({
            'success': True,
            'design_name': design.name,
            'reference_instance': state.reference_instance,
            'chiplet_defs': chiplet_defs,
            'instances': instances,
            'connections': connections,
            'score': score_data
        })
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Update instance (drag, rotate, flip)
# ------------------------------------------------------------------

@app.route('/api/update_instance', methods=['POST'])
def update_instance():
    try:
        if not state.design:
            return jsonify({'success': False, 'error': 'No design loaded'})
        
        data = request.get_json()
        inst_name = data.get('name', '')
        inst = state.design.get_instance(inst_name)
        if not inst:
            return jsonify({'success': False, 'error': f'Instance {inst_name} not found'})
        
        # Reference instance cannot be moved/rotated/flipped by user (only Compaction)
        is_ref = (inst_name == state.reference_instance)
        
        if 'x' in data:
            if is_ref:
                return jsonify({'success': False, 'error': f'Cannot move reference instance {inst_name}'})
            inst.pose.x = float(data['x'])
        if 'y' in data:
            if is_ref:
                return jsonify({'success': False, 'error': f'Cannot move reference instance {inst_name}'})
            inst.pose.y = float(data['y'])
        if 'z' in data:
            inst.pose.z = float(data['z'])
        if 'orientation' in data:
            if is_ref:
                return jsonify({'success': False, 'error': f'Cannot rotate reference instance {inst_name}'})
            inst.pose.orientation = data['orientation']
        if 'flip' in data:
            if is_ref:
                return jsonify({'success': False, 'error': f'Cannot flip reference instance {inst_name}'})
            inst.pose.flip = data['flip']
        if 'mz' in data:
            if is_ref:
                return jsonify({'success': False, 'error': f'Cannot flip reference instance {inst_name}'})
            inst.pose.mz = bool(data['mz'])
        if 'visible' in data:
            inst.visible = bool(data['visible'])
        
        # Invalidate cached solution
        state.solution = None
        
        return jsonify({'success': True})
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Set reference instance
# ------------------------------------------------------------------

@app.route('/api/set_reference', methods=['POST'])
def set_reference():
    try:
        if not state.design:
            return jsonify({'success': False, 'error': 'No design loaded'})
        
        data = request.get_json()
        name = data.get('name', '')
        if not name:
            state.reference_instance = ""
            return jsonify({'success': True, 'reference': ""})
        
        inst = state.design.get_instance(name)
        if not inst:
            return jsonify({'success': False, 'error': f'Instance {name} not found'})
        
        state.reference_instance = name
        return jsonify({'success': True, 'reference': name})
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Run coarse-placement
# ------------------------------------------------------------------

@app.route('/api/run_placement', methods=['POST'])
def run_placement():
    try:
        if not state.design:
            return jsonify({'success': False, 'error': 'No design loaded'})
        
        data = request.get_json() or {}
        
        # Override config from request
        enclosure = data.get('enclosure', state.config['enclosure'])
        sa_iterations = data.get('sa_iterations', state.config['sa_iterations'])
        algorithm = data.get('algorithm', 'SA')  # "SA" or "Expert"
        
        design = state.design
        
        placer = Placer(design, algorithm=algorithm, sa_iterations=sa_iterations, enclosure=enclosure)
        solution = placer.solve()
        state.solution = solution
        
        checker = ConstraintChecker(design)
        
        return jsonify({
            'success': True,
            'score': solution.score,
            'valid': solution.report.is_valid,
            'interposer_size': solution.interposer_size,
            'hard_violations': solution.report.hard_violations,
            'soft_scores': solution.report.soft_scores,
            'score_details': solution.report.score_details,
            'weights': checker.weights,
            'algorithm': algorithm
        })
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Upload D2D connection file
# ------------------------------------------------------------------

@app.route('/api/upload_connection', methods=['POST'])
def upload_connection():
    try:
        if not state.design:
            return jsonify({'success': False, 'error': 'No design loaded'})
        
        data = request.get_json() or {}
        content = data.get('content', '')
        filename = data.get('filename', 'D2D.connection')
        
        # Parse connection content directly
        connections = []
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) >= 2:
                source = parts[0].strip()
                target = parts[1].strip()
                lsi = parts[2].strip() if len(parts) >= 3 else ""
                src_parts = source.split('.')
                tgt_parts = target.split('.')
                src_inst = src_parts[0] if len(src_parts) > 0 else source
                src_ip = src_parts[1] if len(src_parts) > 1 else ""
                tgt_inst = tgt_parts[0] if len(tgt_parts) > 0 else target
                tgt_ip = tgt_parts[1] if len(tgt_parts) > 1 else ""
                connections.append(D2DConnection(
                    source_inst=src_inst,
                    source_ip=src_ip,
                    target_inst=tgt_inst,
                    target_ip=tgt_ip,
                    lsi_inst=lsi,
                    is_external="~" in line
                ))
        
        state.design.d2d_connections = connections
        return jsonify({
            'success': True,
            'connection_count': len(connections)
        })
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Generate LSI
# ------------------------------------------------------------------

@app.route('/api/generate_lsi', methods=['POST'])
def generate_lsi():
    try:
        if not state.design:
            return jsonify({'success': False, 'error': 'No design loaded'})
        
        data = request.get_json() or {}
        conn_idx = data.get('connection_index', 0)
        lsi_name = data.get('lsi_name', 'LSI_auto')
        
        design = state.design
        if conn_idx >= len(design.d2d_connections):
            return jsonify({'success': False, 'error': 'Invalid connection index'})
        
        conn = design.d2d_connections[conn_idx]
        router = D2DRouter(design)
        lsi_def = router.generate_lsi(conn, lsi_name)
        
        if lsi_def:
            design.chiplet_defs[lsi_def.name] = lsi_def
            return jsonify({
                'success': True,
                'lsi_name': lsi_def.name,
                'lsi_size': lsi_def.size
            })
        else:
            return jsonify({'success': False, 'error': 'Could not generate LSI'})
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Compaction
# ------------------------------------------------------------------

@app.route('/api/compaction', methods=['POST'])
def compaction():
    try:
        if not state.design:
            return jsonify({'success': False, 'error': 'No design loaded'})
        
        data = request.get_json() or {}
        enclosure = data.get('enclosure', state.config['enclosure'])
        
        compactor = Compaction(state.design, min_enclosure=enclosure)
        compactor.update_interposer()
        w, h = compactor.compute_interposer_size()
        ox, oy = compactor.compute_interposer_origin()
        
        return jsonify({
            'success': True,
            'interposer_size': [w, h],
            'interposer_origin': [ox, oy]
        })
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Export 3Dblox (returns content for browser download)
# ------------------------------------------------------------------

@app.route('/api/export', methods=['POST'])
def export():
    try:
        if not state.design:
            return jsonify({'success': False, 'error': 'No design loaded'})
        
        # Apply current solution if available
        if state.solution:
            state.solution.apply_to_design()
        
        design = state.design
        
        # Build a fresh solution for export
        checker = ConstraintChecker(design)
        report = checker.check_all()
        
        from chiplets_floorplan.core.models import PlacementSolution
        solution = PlacementSolution(
            design=design,
            instance_poses={inst.name: inst.pose.copy() for inst in design.instances},
            interposer_size=(design.interposer.size if design.interposer else (0, 0)),
            score=report.total_score,
            report=report
        )
        
        exporter = Exporter(solution)
        
        # Export to a temporary directory to read file contents
        import tempfile
        output_dir = tempfile.mkdtemp(prefix='chiplets_export_')
        exported_paths = exporter.export(output_dir, design.name)
        
        # Read file contents for browser download
        file_contents = {}
        for path in exported_paths:
            if os.path.exists(path):
                with open(path, 'r', encoding='utf-8') as f:
                    file_contents[os.path.basename(path)] = f.read()
        
        # Cleanup temp directory
        import shutil
        shutil.rmtree(output_dir, ignore_errors=True)
        
        return jsonify({
            'success': True,
            'files': file_contents,
            'design_name': design.name or 'design'
        })
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------
# Load design from uploaded content (no tkinter needed)
# ------------------------------------------------------------------

@app.route('/api/load_design_content', methods=['POST'])
def load_design_content():
    try:
        data = request.get_json() or {}
        dbx_content = data.get('dbx_content', '')
        connection_content = data.get('connection_content', '')
        
        if not dbx_content:
            return jsonify({'success': False, 'error': 'No .3dbx content provided'})
        
        base_dir = _find_base_dir_for_includes(dbx_content)
        parser = Parser()
        design = parser.parse_design_from_content(dbx_content, base_dir=base_dir)
        
        # Parse D2D connections if provided
        if connection_content:
            design.d2d_connections = parser.parse_connections(connection_content)
        
        state.design = design
        state.dbx_path = 'uploaded'
        state.solution = None
        
        # Default reference: Interposer if exists
        state.reference_instance = ''
        for inst in design.instances:
            if inst.reference == 'Interposer':
                state.reference_instance = inst.name
                break
        
        chiplet_names = [inst.name for inst in design.instances]
        chiplet_types = sorted(set(design.chiplet_defs.keys()))
        
        return jsonify({
            'success': True,
            'design_name': design.name,
            'chiplet_count': len(design.chiplet_defs),
            'instance_count': len(design.instances),
            'd2d_connections': len(design.d2d_connections),
            'chiplet_names': chiplet_names,
            'chiplet_types': chiplet_types
        })
    except Exception as e:
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)})

# ------------------------------------------------------------------

if __name__ == '__main__':
    print("Starting 3D IC Chiplets Coarse-Placement System Web Interface...")
    print("Server will be available at: http://localhost:5000")
    print("Press Ctrl+C to stop the server")
    print()
    app.run(debug=True, host='0.0.0.0', port=5000)
