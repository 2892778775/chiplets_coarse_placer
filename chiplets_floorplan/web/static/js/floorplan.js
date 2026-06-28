
/**
 * 3D IC Chiplets Floorplan System - Frontend Controller
 * Handles Canvas rendering, user interaction, API calls, and constraint visualization.
 */

let chipletInstances = [];
let chipletDefs = {};
let d2dConnections = [];
let selectedInstances = new Set();
let lastClickTime = 0;
let lastClickTarget = null;
let dragStartPositions = {};
let canvas, ctx;
let zoom = 0.005;
let offsetX = 100;
let offsetY = 100;
let hasSetInitialOffset = false;
let isDragging = false;
let dragStartX = 0;
let dragStartY = 0;
let dragStartInstX = 0;
let dragStartInstY = 0;
let scoreData = null;
let referenceInstance = "";

const TYPE_COLORS = {
    'SOC': '#3498db',
    'HBM': '#2ecc71',
    'Interposer': '#e0e0e0',
    'LSI': '#f39c12',
    'LSI1': '#e67e22',
    'LSI2': '#d35400',
    'Dummy': '#bdc3c7'
};

const HARD_VIOLATION_COLOR = 'rgba(231, 76, 60, 0.3)';
const D2D_LINE_COLOR = '#e74c3c';
const D2D_LINE_WIDTH = 2;

document.addEventListener('DOMContentLoaded', function() {
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);
    canvas.addEventListener('mousedown', canvasMouseDown);
    canvas.addEventListener('mousemove', canvasMouseMove);
    canvas.addEventListener('mouseup', canvasMouseUp);
    canvas.addEventListener('mouseleave', canvasMouseUp);
    canvas.addEventListener('wheel', canvasWheel);
    setInterval(refreshData, 5000);
});

function resizeCanvas() {
    const container = canvas.parentElement;
    canvas.width = container.clientWidth;
    canvas.height = container.clientHeight;
    // Place initial origin in bottom-left area of canvas
    if (!hasSetInitialOffset) {
        offsetX = 50;
        offsetY = canvas.height - 50;
        hasSetInitialOffset = true;
    }
    drawCanvas();
}

function drawCanvas() {
    if (!ctx) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    drawGrid();
    if (chipletInstances.length === 0) {
        ctx.fillStyle = '#999';
        ctx.font = '16px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('No chiplets loaded. Click "Import 3DBX" to load a file.', canvas.width / 2, canvas.height / 2);
        return;
    }
    drawD2DConnections();
    // Sort by z ascending: smaller z drawn first (bottom layer), larger z drawn last (top layer)
    const sortedInstances = [...chipletInstances].sort((a, b) => a.z - b.z);
    const interposer = sortedInstances.find(i => i.module === 'Interposer');
    if (interposer) drawChiplet(interposer, true);
    for (const inst of sortedInstances) {
        if (inst.module === 'Interposer') continue;
        if (!inst.visible) continue;
        drawChiplet(inst, false);
    }
    drawViolationHighlights();
    drawScoreOverlay();
}

function getReferenceOrigin() {
    const ref = chipletInstances.find(i => i.name === referenceInstance);
    return { refX: ref ? ref.x : 0, refY: ref ? ref.y : 0 };
}

function getTransformedBounds(w, h, orientation, flip) {
    const corners = [[0, 0], [w, 0], [0, h], [w, h]];
    let transformed = corners.map(([x, y]) => [x, y]);
    // flip around origin (left-bottom corner)
    if (flip === 'MX') {
        transformed = transformed.map(([x, y]) => [x, -y]);
    } else if (flip === 'MY') {
        transformed = transformed.map(([x, y]) => [-x, y]);
    }
    // rotate around origin (left-bottom corner), CCW
    let angle = 0;
    if (orientation === 'R90') angle = Math.PI / 2;
    else if (orientation === 'R180') angle = Math.PI;
    else if (orientation === 'R270') angle = -Math.PI / 2;
    const cos = Math.cos(angle), sin = Math.sin(angle);
    transformed = transformed.map(([x, y]) => [
        x * cos - y * sin,
        x * sin + y * cos
    ]);
    const minX = Math.min(...transformed.map(p => p[0]));
    const minY = Math.min(...transformed.map(p => p[1]));
    const maxX = Math.max(...transformed.map(p => p[0]));
    const maxY = Math.max(...transformed.map(p => p[1]));
    return { minX, minY, maxX, maxY, width: maxX - minX, height: maxY - minY };
}

function drawGrid() {
    const { refX, refY } = getReferenceOrigin();
    const gridSize = 50 * zoom;
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 1;
    // Vertical grid lines: align with reference origin X
    const gridBaseX = offsetX - refX * zoom;
    let startX = gridBaseX % gridSize;
    if (startX < 0) startX += gridSize;
    for (let x = startX; x < canvas.width; x += gridSize) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
    }
    // Horizontal grid lines: align with reference origin Y
    const gridBaseY = offsetY + refY * zoom;
    let startY = gridBaseY % gridSize;
    if (startY < 0) startY += gridSize;
    for (let y = startY; y < canvas.height; y += gridSize) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
    }
    // Draw origin cross at reference chiplet's bottom-left
    const ox = offsetX;
    const oy = offsetY;
    ctx.strokeStyle = '#ff6b6b'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(ox - 10, oy); ctx.lineTo(ox + 10, oy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(ox, oy - 10); ctx.lineTo(ox, oy + 10); ctx.stroke();
    ctx.fillStyle = '#ff6b6b'; ctx.font = 'bold 12px Arial';
    ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    ctx.fillText('(0,0)', ox + 5, oy - 5);
}

function drawChiplet(inst, isInterposer) {
    const def = chipletDefs[inst.module];
    if (!def) return;
    const { refX, refY } = getReferenceOrigin();
    
    const w = def.size[0] * zoom;
    const h = def.size[1] * zoom;
    const bounds = getTransformedBounds(w, h, inst.orientation, inst.flip);
    
    // Bounding box left-bottom in screen coordinates
    // user (inst.x, inst.y) -> screen (offsetX + (inst.x - refX)*zoom, offsetY - (inst.y - refY)*zoom)
    const cornerX = offsetX + (inst.x - refX) * zoom;
    const cornerY = offsetY - (inst.y - refY) * zoom;
    
    ctx.save();
    // Move to the bounding box's left-bottom corner in screen space
    ctx.translate(cornerX, cornerY);
    // Flip Y so that in local coordinates Y points UP (matches user coordinate system)
    ctx.scale(1, -1);
    // Translate so that the transformed bounding box's left-bottom is at (0,0)
    ctx.translate(-bounds.minX, -bounds.minY);
    // Rotate CCW around the chiplet's left-bottom corner (0,0 in local coords)
    let rotation = 0;
    if (inst.orientation === 'R90') rotation = Math.PI / 2;
    else if (inst.orientation === 'R180') rotation = Math.PI;
    else if (inst.orientation === 'R270') rotation = -Math.PI / 2;
    ctx.rotate(rotation);
    // Flip in user coordinates: MX = flip across X-axis (y -> -y)
    // Since we already scaled Y to point UP, MX in user space = scale(1, -1) in this local space
    if (inst.flip === 'MX') ctx.scale(1, -1);
    if (inst.flip === 'MY') ctx.scale(-1, 1);
    
    let color = TYPE_COLORS[inst.module] || '#9b59b6';
    for (const [key, value] of Object.entries(TYPE_COLORS)) {
        if (inst.module.startsWith(key)) { color = value; break; }
    }
    if (isInterposer) color = '#f0f0f0';
    
    // Reference instance highlight
    if (inst.name === referenceInstance) {
        ctx.strokeStyle = '#f39c12'; ctx.lineWidth = 4;
        ctx.strokeRect(-2, -2, w + 4, h + 4);
    }
    // Selected highlight
    if (selectedInstances.has(inst.name)) {
        ctx.strokeStyle = '#e74c3c'; ctx.lineWidth = 3;
        ctx.strokeRect(-2, -2, w + 4, h + 4);
    }
    
    // Draw chiplet rectangle from (0,0) to (w,h) in local coordinates
    // (0,0) is the chiplet's left-bottom corner
    ctx.fillStyle = color; ctx.globalAlpha = isInterposer ? 0.5 : 0.85;
    ctx.fillRect(0, 0, w, h); ctx.globalAlpha = 1;
    ctx.strokeStyle = isInterposer ? '#cccccc' : '#2c3e50'; ctx.lineWidth = isInterposer ? 1 : 2;
    ctx.strokeRect(0, 0, w, h);
    
    // Face-down (MZ) indicator: diagonal cross-hatch pattern
    if (!isInterposer && inst.mz) {
        ctx.save();
        ctx.strokeStyle = 'rgba(0,0,0,0.3)';
        ctx.lineWidth = 1;
        const step = 10 * zoom;
        ctx.beginPath();
        for (let x = -h; x < w + h; x += step) {
            ctx.moveTo(x, 0);
            ctx.lineTo(x + h, h);
        }
        ctx.stroke();
        ctx.restore();
    }
    
    // Draw full instance name centered in the chiplet
    // Text needs to be flipped back to be readable
    if (!isInterposer && w > 30 && h > 20) {
        ctx.save();
        ctx.translate(w / 2, h / 2);
        ctx.scale(1, -1); // Flip Y back so text is readable
        ctx.fillStyle = '#ffffff';
        ctx.font = `${Math.max(8, 12 * zoom)}px Arial`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(inst.name, 0, 0);
        ctx.restore();
    }
    
    // Draw IPs (D2D PHYs) inside this chiplet's local coordinate system
    // so they rotate/flip together with the parent chiplet
    if (!isInterposer && inst.ips && inst.ips.length > 0) {
        for (const ip of inst.ips) {
            const ipX = ip.local_x * zoom;
            const ipY = ip.local_y * zoom;
            const ipW = ip.size[0] * zoom;
            const ipH = ip.size[1] * zoom;
            ctx.fillStyle = '#e74c3c';
            ctx.globalAlpha = 0.9;
            ctx.fillRect(ipX, ipY, ipW, ipH);
            ctx.globalAlpha = 1;
            ctx.strokeStyle = '#c0392b';
            ctx.lineWidth = 1;
            ctx.strokeRect(ipX, ipY, ipW, ipH);
        }
    }
    
    ctx.restore();
}

function drawD2DConnections() {
    if (!d2dConnections || d2dConnections.length === 0) return;
    const { refX, refY } = getReferenceOrigin();
    ctx.strokeStyle = D2D_LINE_COLOR; ctx.lineWidth = D2D_LINE_WIDTH;
    for (const conn of d2dConnections) {
        const sx = offsetX + (conn.source_x - refX) * zoom;
        const sy = offsetY - (conn.source_y - refY) * zoom;
        const tx = offsetX + (conn.target_x - refX) * zoom;
        const ty = offsetY - (conn.target_y - refY) * zoom;
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(tx, ty); ctx.stroke();
        ctx.fillStyle = D2D_LINE_COLOR;
        ctx.beginPath(); ctx.arc(sx, sy, 3, 0, Math.PI * 2); ctx.fill();
        ctx.beginPath(); ctx.arc(tx, ty, 3, 0, Math.PI * 2); ctx.fill();
    }
}

function drawViolationHighlights() {
    if (!scoreData || !scoreData.hard_violations) return;
    const { refX, refY } = getReferenceOrigin();
    for (const v of scoreData.hard_violations) {
        if (v.startsWith('H1:')) {
            const parts = v.split(' ');
            const inst1 = parts[2], inst2 = parts[4];
            const i1 = chipletInstances.find(i => i.name === inst1);
            const i2 = chipletInstances.find(i => i.name === inst2);
            if (i1 && i2) {
                const def1 = chipletDefs[i1.module];
                if (def1) {
                    const w = def1.size[0] * zoom;
                    const h = def1.size[1] * zoom;
                    const bounds = getTransformedBounds(w, h, i1.orientation, i1.flip);
                    ctx.fillStyle = HARD_VIOLATION_COLOR;
                    const x = offsetX + (i1.x - refX) * zoom;
                    const y = offsetY - (i1.y + bounds.height - refY) * zoom;
                    ctx.fillRect(x, y, bounds.width, bounds.height);
                }
            }
        }
    }
}

function drawScoreOverlay() {
    if (!scoreData) return;
    ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
    ctx.fillRect(10, 10, 200, 80);
    ctx.strokeStyle = '#ccc'; ctx.lineWidth = 1; ctx.strokeRect(10, 10, 200, 80);
    ctx.fillStyle = scoreData.valid ? '#27ae60' : '#e74c3c';
    ctx.font = 'bold 14px Arial'; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(`Score: ${scoreData.total?.toFixed(3) || 'N/A'}`, 18, 18);
    ctx.fillStyle = '#333'; ctx.font = '12px Arial';
    ctx.fillText(`Valid: ${scoreData.valid ? 'YES' : 'NO'}`, 18, 40);
    if (scoreData.hard_violations && scoreData.hard_violations.length > 0) {
        ctx.fillStyle = '#e74c3c'; ctx.fillText(`Violations: ${scoreData.hard_violations.length}`, 18, 58);
    }
}

function canvasMouseDown(e) {
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left, y = e.clientY - rect.top;
    const now = Date.now();
    const clickedInst = findInstanceAt(x, y);
    const isDoubleClick = (now - lastClickTime < 300) && (lastClickTarget === clickedInst);
    lastClickTime = now;
    lastClickTarget = clickedInst;

    if (clickedInst) {
        // Double-click: toggle selection
        if (isDoubleClick) {
            if (selectedInstances.has(clickedInst)) {
                selectedInstances.delete(clickedInst);
            } else {
                selectedInstances.add(clickedInst);
            }
            drawCanvas(); updateTables(); return;
        }
        // Single-click: start drag if part of selection, or select new if none selected
        if (!selectedInstances.has(clickedInst) && selectedInstances.size > 0) {
            selectedInstances.clear();
            selectedInstances.add(clickedInst);
        } else if (selectedInstances.size === 0) {
            selectedInstances.add(clickedInst);
        }
        // Start drag (skip reference instance)
        dragStartPositions = {};
        for (const name of selectedInstances) {
            const inst = chipletInstances.find(i => i.name === name);
            if (inst && inst.name !== referenceInstance) {
                dragStartPositions[name] = { x: inst.x, y: inst.y };
            }
        }
        if (Object.keys(dragStartPositions).length > 0) {
            isDragging = true; dragStartX = x; dragStartY = y;
            canvas.style.cursor = 'grabbing';
        }
        drawCanvas(); updateTables();
    } else {
        // Click on empty area: clear all selection
        if (selectedInstances.size > 0) {
            selectedInstances.clear();
            drawCanvas(); updateTables();
        }
    }
}

function canvasMouseMove(e) {
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left, y = e.clientY - rect.top;
    if (isDragging && selectedInstances.size > 0) {
        const deltaX = (x - dragStartX) / zoom;
        const deltaY = -(y - dragStartY) / zoom; // Y-flip: screen down = user Y decreases
        const promises = [];
        for (const [name, startPos] of Object.entries(dragStartPositions)) {
            promises.push(fetch('/api/update_instance', {
                method: 'POST', headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name: name, x: startPos.x + deltaX, y: startPos.y + deltaY })
            }));
        }
        Promise.all(promises).then(() => refreshData());
        canvas.style.cursor = 'grabbing'; return;
    }
    canvas.style.cursor = findInstanceAt(x, y) ? 'grab' : 'crosshair';
}

function canvasMouseUp() { isDragging = false; canvas.style.cursor = 'crosshair'; }

function canvasWheel(e) {
    e.preventDefault();
    const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
    zoom = Math.max(0.0001, Math.min(0.1, zoom * zoomFactor));
    drawCanvas();
}

function findInstanceAt(canvasX, canvasY) {
    // Sort by z descending: larger z (top layer) checked first
    const sorted = [...chipletInstances].sort((a, b) => b.z - a.z);
    const { refX, refY } = getReferenceOrigin();
    for (const inst of sorted) {
        if (!inst.visible) continue;
        const def = chipletDefs[inst.module];
        if (!def) continue;
        const w = def.size[0] * zoom;
        const h = def.size[1] * zoom;
        const bounds = getTransformedBounds(w, h, inst.orientation, inst.flip);
        const x = offsetX + (inst.x - refX) * zoom;
        const y = offsetY - (inst.y + bounds.height - refY) * zoom;
        if (canvasX >= x && canvasX <= x + bounds.width && canvasY >= y && canvasY <= y + bounds.height) return inst.name;
    }
    return null;
}

function onDbxFileSelected(input) {
    const file = input.files[0];
    if (!file) return;
    showMessage('Reading ' + file.name + '...', 'info');
    const reader = new FileReader();
    reader.onload = function(e) {
        const dbxContent = e.target.result;
        const payload = { dbx_content: dbxContent };
        if (pendingConnectionContent) {
            payload.connection_content = pendingConnectionContent;
        }
        fetch('/api/load_design_content', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        }).then(r => {
            if (!r.ok) throw new Error('HTTP ' + r.status);
            return r.json();
        }).then(data => {
            if (data.success) {
                document.getElementById('filePathDisplay').textContent = file.name;
                showMessage('Loaded ' + data.instance_count + ' instances', 'success');
                refreshData();
            } else {
                showMessage(data.error || 'Failed to load design', 'error');
            }
        }).catch(err => {
            console.error('Load design failed:', err);
            showMessage('Load design failed: ' + err.message, 'error');
        });
    };
    reader.onerror = function() {
        showMessage('Failed to read file', 'error');
    };
    reader.readAsText(file);
}

function refreshData() {
    fetch('/api/get_data').then(r => {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
    }).then(data => {
        if (data.success) {
            chipletInstances = data.instances || [];
            chipletDefs = {};
            for (const def of data.chiplet_defs || []) chipletDefs[def.name] = def;
            d2dConnections = data.connections || [];
            scoreData = data.score;
            referenceInstance = data.reference_instance || '';
            document.getElementById('design-name').textContent = 'Design: ' + (data.design_name || 'None');
            updateTables(); drawCanvas();
        }
    }).catch(err => {
        console.error('refreshData failed:', err);
        showMessage('Refresh failed: ' + err.message, 'error');
    });
}

function runPlacement() {
    const algorithm = document.getElementById('placementAlgorithm').value;
    showMessage('Running ' + algorithm + ' placement...', 'info');
    fetch('/api/run_placement', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ algorithm: algorithm }) })
        .then(r => {
            if (!r.ok) throw new Error('HTTP ' + r.status + ': ' + r.statusText);
            return r.json();
        })
        .then(data => {
            if (data.success) {
                showMessage(algorithm + ' Score: ' + data.score.toFixed(3), 'success');
                refreshData();
            } else {
                showMessage(data.error || 'Placement failed', 'error');
            }
        })
        .catch(err => {
            console.error('runPlacement failed:', err);
            showMessage('Placement failed: ' + err.message, 'error');
        });
}

function calculateDummy() {
    showMessage('Calculating dummy dies...', 'info');
    fetch('/api/calculate_dummy', { method: 'POST', headers: { 'Content-Type': 'application/json' } })
        .then(r => r.json()).then(data => {
            if (data.success) { showMessage(data.dummy_count + ' dummies created', 'success'); refreshData(); }
            else showMessage(data.error, 'error');
        });
}

function doCompaction() {
    showMessage('Running compaction...', 'info');
    fetch('/api/compaction', { method: 'POST', headers: { 'Content-Type': 'application/json' } })
        .then(r => r.json()).then(data => {
            if (data.success) { showMessage('Interposer: ' + data.interposer_size[0].toFixed(0) + ' x ' + data.interposer_size[1].toFixed(0), 'success'); refreshData(); }
            else showMessage(data.error, 'error');
        });
}

async function exportDesign() {
    showMessage('Exporting design...', 'info');
    
    try {
        // Use File System Access API to let user pick a save directory
        let dirHandle;
        try {
            dirHandle = await window.showDirectoryPicker();
        } catch (e) {
            if (e.name === 'AbortError') {
                showMessage('Export cancelled', 'info');
                return;
            }
            throw new Error('Directory picker not supported or denied by browser');
        }
        
        // Get file contents from backend
        const response = await fetch('/api/export', { 
            method: 'POST', 
            headers: { 'Content-Type': 'application/json' } 
        });
        if (!response.ok) throw new Error('HTTP ' + response.status);
        
        const data = await response.json();
        if (!data.success || !data.files) {
            throw new Error(data.error || 'Export failed');
        }
        
        // Write each file directly into the chosen directory
        let count = 0;
        for (const [filename, content] of Object.entries(data.files)) {
            const fileHandle = await dirHandle.getFileHandle(filename, { create: true });
            const writable = await fileHandle.createWritable();
            await writable.write(content);
            await writable.close();
            count++;
        }
        
        showMessage('Exported ' + count + ' files', 'success');
    } catch (err) {
        console.error('Export failed:', err);
        showMessage('Export failed: ' + err.message + '. Falling back to download...', 'error');
        
        // Fallback: direct download
        fallbackExportDownload();
    }
}

function fallbackExportDownload() {
    fetch('/api/export', { method: 'POST', headers: { 'Content-Type': 'application/json' } })
        .then(r => {
            if (!r.ok) throw new Error('HTTP ' + r.status);
            return r.json();
        })
        .then(data => {
            if (data.success && data.files) {
                let count = 0;
                for (const [filename, content] of Object.entries(data.files)) {
                    const blob = new Blob([content], { type: 'text/plain' });
                    const url = URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = filename;
                    document.body.appendChild(a);
                    a.click();
                    document.body.removeChild(a);
                    URL.revokeObjectURL(url);
                    count++;
                }
                showMessage('Downloaded ' + count + ' files', 'success');
            } else {
                showMessage(data.error || 'Export failed', 'error');
            }
        })
        .catch(err => {
            console.error('Fallback export failed:', err);
            showMessage('Fallback export failed: ' + err.message, 'error');
        });
}

let pendingConnectionContent = '';
let pendingConnectionName = '';

function onConnectionSelected(input) {
    const file = input.files[0];
    if (!file) return;
    pendingConnectionName = file.name;
    const reader = new FileReader();
    reader.onload = function(e) {
        pendingConnectionContent = e.target.result;
        document.getElementById('connectionPathDisplay').textContent = file.name;
        
        if (chipletInstances.length > 0) {
            fetch('/api/upload_connection', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ content: pendingConnectionContent, filename: file.name })
            }).then(r => r.json()).then(data => {
                if (data.success) {
                    showMessage(data.connection_count + ' D2D connections loaded', 'success');
                    refreshData();
                } else {
                    showMessage(data.error, 'error');
                }
            }).catch(err => {
                showMessage('Connection upload failed: ' + err.message, 'error');
            });
        } else {
            showMessage('D2D.connection selected: ' + file.name + '. Click "Import 3DBX" to load together.', 'info');
        }
    };
    reader.readAsText(file);
}

function rotateSelected(angle) {
    if (selectedInstances.size === 0) { showMessage('Select a chiplet first', 'error'); return; }
    const orientations = ['R0', 'R90', 'R180', 'R270'];
    const promises = [];
    for (const name of selectedInstances) {
        const inst = chipletInstances.find(i => i.name === name);
        if (!inst || inst.name === referenceInstance) continue;
        const idx = orientations.indexOf(inst.orientation);
        const newIdx = (idx + (angle > 0 ? 1 : -1) + 4) % 4;
        inst.orientation = orientations[newIdx]; // update local
        promises.push(fetch('/api/update_instance', {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: name, orientation: orientations[newIdx] })
        }));
    }
    if (promises.length > 0) Promise.all(promises);
}

function flipSelected(flipType) {
    if (selectedInstances.size === 0) { showMessage('Select a chiplet first', 'error'); return; }
    const promises = [];
    for (const name of selectedInstances) {
        const inst = chipletInstances.find(i => i.name === name);
        if (!inst || inst.name === referenceInstance) continue;
        if (flipType === 'MZ') {
            // Z-axis flip: toggle mz boolean
            inst.mz = !inst.mz;
            promises.push(fetch('/api/update_instance', {
                method: 'POST', headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name: name, mz: inst.mz })
            }));
        } else {
            const newFlip = inst.flip === flipType ? 'None' : flipType;
            inst.flip = newFlip; // update local
            promises.push(fetch('/api/update_instance', {
                method: 'POST', headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name: name, flip: newFlip })
            }));
        }
    }
    if (promises.length > 0) Promise.all(promises).then(() => drawCanvas());
}

function deleteSelected() {
    if (selectedInstances.size === 0) { showMessage('Select a chiplet first', 'error'); return; }
    const promises = [];
    for (const name of selectedInstances) {
        // Update local state
        const inst = chipletInstances.find(i => i.name === name);
        if (inst) inst.visible = false;
        promises.push(fetch('/api/update_instance', {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: name, visible: false })
        }));
    }
    selectedInstances.clear();
    Promise.all(promises);
}

function updateTables() { updateDefsTable(); updateInstancesTable(); updateScorePanel(); }

function updateDefsTable() {
    const tbody = document.querySelector('#chiplet-defs-table tbody');
    if (!tbody) return; tbody.innerHTML = '';
    for (const [name, def] of Object.entries(chipletDefs)) {
        const row = document.createElement('tr');
        row.innerHTML = `<td>${name}</td><td>(${def.size[0]}, ${def.size[1]})</td><td>${def.thickness}</td><td>${def.shrink}</td><td>${def.seal_ring?.join(',')}</td><td>${def.scribe_line?.join(',')}</td>`;
        tbody.appendChild(row);
    }
}

function updateInstancesTable() {
    const tbody = document.querySelector('#chiplet-instances-table tbody');
    if (!tbody) return; tbody.innerHTML = '';
    for (const inst of chipletInstances) {
        const row = document.createElement('tr');
        if (selectedInstances.has(inst.name)) row.style.backgroundColor = '#e3f2fd';
        const isRef = (inst.name === referenceInstance);
        const refBadge = isRef ? ' <span title="Reference (coordinate system origin)">⭐</span>' : '';
        const refBtn = isRef ? '<span>⭐</span>' : `<button onclick="setReference('${inst.name}')">Set Ref</button>`;
        const disabledAttr = isRef ? 'disabled' : '';
        row.innerHTML = `<td><input type="checkbox" ${inst.visible ? 'checked' : ''} onchange="toggleVisible('${inst.name}', this.checked)"></td><td>${inst.name}${refBadge}</td><td>${inst.module}</td><td>${inst.group || ''}</td><td>${refBtn}</td><td><input type="number" value="${inst.x.toFixed(0)}" onchange="updateInstCoord('${inst.name}', 'x', this.value)" ${disabledAttr}></td><td><input type="number" value="${inst.y.toFixed(0)}" onchange="updateInstCoord('${inst.name}', 'y', this.value)" ${disabledAttr}></td><td><input type="number" value="${inst.z}" onchange="updateInstCoord('${inst.name}', 'z', this.value)"></td><td>${inst.flip}</td><td>${inst.mz ? '▼' : '▲'}</td><td>${inst.orientation}</td>`;
        tbody.appendChild(row);
    }
}

function setReference(name) {
    fetch('/api/set_reference', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name: name }) }).then(() => refreshData());
}

function updateScorePanel() {
    const panel = document.getElementById('score-panel');
    if (!panel) return;
    if (!scoreData) { panel.innerHTML = '<p>No score data. Run placement to see score.</p>'; return; }
    
    try {
        let html = '<table class="data-table score-formula-table" style="width:100%;">';
        html += '<thead><tr><th>Metrics</th><th>Formula</th><th>Variable Description</th><th>Variable Values</th><th>Score</th></tr></thead>';
        html += '<tbody>';
        
        const ruleNames = {
            'hbm_placement': 'HBM Placement',
            'soc_center': 'SOC Center',
            'horizontal_symmetry': 'Horizontal Symmetry',
            'interposer_area': 'Interposer Area',
            'aspect_ratio': 'Aspect Ratio',
            'd2d_length': 'D2D Length',
            'dummy_minimize': 'Dummy Minimize'
        };
        
        // Soft rules with formula details
        if (scoreData.score_details) {
            for (const [key, detail] of Object.entries(scoreData.score_details)) {
                try {
                    const name = ruleNames[key] || key;
                    const formula = (detail && detail.formula) ? detail.formula : 'N/A';
                    const score = (detail && detail.score !== undefined && detail.score !== null) ? Number(detail.score).toFixed(4) : 'N/A';
                    
                    let varDesc = '';
                    let varVals = '';
                    
                    if (key === 'hbm_placement' && detail.vars) {
                        for (const [instName, v] of Object.entries(detail.vars)) {
                            varDesc += `${instName}: cx=center_x, dist_to_left/right=distance to MBR edge<br>`;
                            varVals += `${instName}: cx=${v.cx}, dist_left=${v.dist_left}, dist_right=${v.dist_right}, min_v=${v.min_v}, max_dim=${v.max_dim}<br>`;
                        }
                    } else if (key === 'soc_center' && detail.vars) {
                        for (const [instName, v] of Object.entries(detail.vars)) {
                            varDesc += `${instName}: cx,cy=SOC center; ip_cx,ip_cy=MBR center<br>`;
                            varVals += `${instName}: cx=${v.cx}, cy=${v.cy}, ip_cx=${v.ip_cx}, ip_cy=${v.ip_cy}, dist=${v.dist}<br>`;
                        }
                    } else if (key === 'horizontal_symmetry' && detail.vars) {
                        for (const [instName, v] of Object.entries(detail.vars)) {
                            varDesc += `${instName}: cx=instance center_x, asymmetry=|cx - center_x|<br>`;
                            varVals += `${instName}: cx=${v.cx}, asymmetry=${v.asymmetry}<br>`;
                        }
                        if (detail.values) {
                            varVals += `avg_asymmetry=${detail.values.avg_asymmetry}, max_offset=${detail.values.max_offset}`;
                        }
                    } else if (key === 'interposer_area' && detail.values) {
                        varDesc = 'MBR_area=MBR bounding box area; total_area=sum of all chiplet areas';
                        varVals = `MBR_area=${detail.values.MBR_area}, total_area=${detail.values.total_area}, max_area=${detail.values.max_area}`;
                    } else if (key === 'aspect_ratio' && detail.values) {
                        varDesc = 'width=MBR width; height=MBR height; ratio=max(h/w, w/h)';
                        varVals = `width=${detail.values.width}, height=${detail.values.height}, ratio=${detail.values.ratio}`;
                    } else if (key === 'd2d_length' && detail.values) {
                        varDesc = 'total_dist=sum of all Manhattan distances; count=number of connections';
                        varVals = `total_dist=${detail.values.total_dist}, count=${detail.values.count}, avg_dist=${detail.values.avg_dist}, max_dist=${detail.values.max_dist}`;
                        if (detail.vars && detail.vars.connections) {
                            for (const c of detail.vars.connections) {
                                varVals += `<br>${c.conn}: dist=${c.manhattan_dist}`;
                            }
                        }
                    } else if (key === 'dummy_minimize' && detail.values) {
                        varDesc = 'dummy_area=total dummy die area; real_area=total real chiplet area';
                        varVals = `dummy_area=${detail.values.dummy_area}, real_area=${detail.values.real_area}, ratio=${detail.values.ratio}`;
                    }
                    
                    html += `<tr><td><b>${name}</b><br><small style="color:#888">weight=${(scoreData.weights && scoreData.weights[key]) || 'N/A'}</small></td>`;
                    html += `<td><code>${formula}</code></td>`;
                    html += `<td><small>${varDesc}</small></td>`;
                    html += `<td><small>${varVals}</small></td>`;
                    const color = (score >= 0.8) ? '#27ae60' : (score >= 0.5) ? '#f39c12' : '#e74c3c';
                    html += `<td style="font-weight:bold;color:${color}">${score}</td></tr>`;
                } catch (rowErr) {
                    console.error(`updateScorePanel row error for ${key}:`, rowErr);
                    html += `<tr><td colspan="5" style="color:#e74c3c">Error rendering ${key}: ${rowErr.message}</td></tr>`;
                }
            }
        }
        
        // Total row
        const totalScore = (scoreData.total !== undefined && scoreData.total !== null) ? Number(scoreData.total).toFixed(4) : 'N/A';
        html += `<tr style="background:#f0f0f0;font-weight:bold"><td colspan="4">Total Score (Weighted Sum)</td><td>${totalScore}</td></tr>`;
        html += '</tbody></table>';
        
        // Hard violations section
        if (scoreData.hard_violations && scoreData.hard_violations.length > 0) {
            html += '<div class="violations" style="margin-top:15px"><h4>Hard Violations:</h4><ul>';
            for (const v of scoreData.hard_violations) html += `<li style="color:#e74c3c">${v}</li>`;
            html += '</ul></div>';
        }
        
        panel.innerHTML = html;
    } catch (e) {
        console.error('updateScorePanel failed:', e);
        panel.innerHTML = `<p style="color:#e74c3c">Error rendering score panel: ${e.message}</p>`;
    }
}

function toggleVisible(name, visible) {
    // Update local state immediately so table shows correct value on re-render
    const inst = chipletInstances.find(i => i.name === name);
    if (inst) inst.visible = visible;
    // Submit to backend but do NOT refresh canvas automatically
    fetch('/api/update_instance', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name, visible }) });
}

function updateInstCoord(name, field, value) {
    const payload = { name }; payload[field] = parseFloat(value);
    // Update local state immediately so table shows correct value on re-render
    const inst = chipletInstances.find(i => i.name === name);
    if (inst) inst[field] = parseFloat(value);
    // Submit to backend but do NOT refresh canvas automatically
    fetch('/api/update_instance', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload) });
}

function showTab(tabName, clickedBtn = null) {
    document.querySelectorAll('.table-tabs button').forEach(btn => btn.classList.remove('active'));
    if (clickedBtn) {
        clickedBtn.classList.add('active');
    } else {
        // Programmatic call: find button by tab name
        const btnMap = { 'chiplet-defs': 0, 'chiplet-instances': 1, 'score': 2 };
        const btns = document.querySelectorAll('.table-tabs button');
        if (btns[btnMap[tabName]]) btns[btnMap[tabName]].classList.add('active');
    }
    document.querySelectorAll('.data-table, .score-panel').forEach(el => el.style.display = 'none');
    const target = document.getElementById(tabName + '-table') || document.getElementById(tabName + '-panel');
    if (target) target.style.display = 'block';
}

function showMessage(message, type) {
    const area = document.getElementById('messageArea');
    area.textContent = message;
    area.style.display = 'block';
    area.className = 'message-area' + (type === 'error' ? ' error' : type === 'success' ? ' success' : '');
    setTimeout(() => { area.style.display = 'none'; }, 4000);
}
