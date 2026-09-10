"""Migration of old asp JSON + STL without importing its Panda3D modules."""
import json
from pathlib import Path
import numpy as np
from ..io import read_mesh, unit_scale
from ..model import Assembly, Part


def legacy_rotation(rotation_rad):
    """Reproduce loader -> CMesh.RPY -> euler_matrix(sxyz): Rz @ Ry @ Rx."""
    r = np.asarray(rotation_rad, dtype=float)
    if r.shape != (3,) or not np.all(np.isfinite(r)):
        raise ValueError('Legacy rotation must contain three finite radians')
    x, y, z = r
    cx, cy, cz = np.cos(r)
    sx, sy, sz = np.sin(r)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def legacy_asset_inventory(legacy_root, name, *, length_unit):
    """Inspect all referenced assets. Missing models remain explicit entries."""
    scale = unit_scale(length_unit)
    root = Path(legacy_root)
    data = json.loads((root / 'asp' / 'data' / name).read_text(encoding='utf-8'))
    entries = []
    for key, value in data.items():
        model = {'smallL 2': 'smallL_2', 'Cross': 'cross', 'Crossleft': 'crossleft'}.get(key, key)
        model = model.replace(' ', '_')
        if 'Domino' in key:
            model = 'domino'
        if 'Alframe' in key:
            model = 'alframe'
        path = root / 'asp' / 'objects' / f'{model}.stl'
        if not path.exists():
            path = path.with_name(model[0].lower() + model[1:] + '.stl')
        tf = np.eye(4)
        tf[:3, :3] = legacy_rotation(value['rotation'])
        tf[:3, 3] = np.asarray(value['location']) * scale
        entry = {'part_id': key, 'path': str(path.resolve()), 'exists': path.is_file(),
                 'assembled_tf_m': tf.tolist(), 'mass_kg': None, 'friction': None}
        if path.is_file():
            mesh = read_mesh(path, length_unit=length_unit)
            entry.update(vertex_count=len(mesh.vertices), triangle_count=len(mesh.faces),
                         extents_m=np.ptp(mesh.vertices, axis=0).tolist())
        entries.append(entry)
    return {'name': name, 'input_length_unit': length_unit,
            'rotation_convention': 'extrinsic xyz radians; Rz @ Ry @ Rx',
            'part_count': len(entries), 'parts': entries,
            'unit_note': 'Explicit user assumption; STL cannot verify its own unit'}


def load_legacy_assembly(legacy_root, name, *, length_unit):
    """Load legacy instances with explicit units; fail on any missing asset."""
    report = legacy_asset_inventory(legacy_root, name, length_unit=length_unit)
    missing = [p['path'] for p in report['parts'] if not p['exists']]
    if missing:
        raise FileNotFoundError('Missing legacy assets: ' + ', '.join(sorted(set(missing))))
    meshes, parts = {}, []
    for p in report['parts']:
        if p['path'] not in meshes:
            meshes[p['path']] = read_mesh(p['path'], length_unit=length_unit)
        parts.append(Part(p['part_id'], meshes[p['path']], p['assembled_tf_m']))
    return Assembly(tuple(parts), provenance=report)
