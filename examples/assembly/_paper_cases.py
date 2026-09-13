"""Paper figure catalog. Original exports and declared reconstructions stay distinct."""
from functools import lru_cache
from itertools import permutations, product
import json
from pathlib import Path
import numpy as np
from wrs.assembly import Assembly, AssemblyState, Part, MeshData
from wrs.assembly.io import read_mesh
from wrs.assembly.adapters.legacy import legacy_rotation
from wrs.assembly.primitives import box, pose

ASSETS = Path(__file__).parent/'assets'/'paper2021'
EDGE = .0185
# Figure keys remain separate even when two experiments reuse one geometry.
# Orders below are inspection prefixes, not newly validated assembly sequences.
CATALOG = {
    'fig08_soma3': ('Fig.8 / Soma 3', 'datainfo2', ('Z','bigL','smallL 2')),
    'fig09_domino3': ('Fig.9 / Domino 3', 'domino_5', ('Domino.001','Domino.002','Domino.003')),
    'fig10_burr6': ('Fig.10 / Burr 6', 'burrpuzzle', ('u','3u','2L right','2L left','2u','1')),
    'fig11_bridge6': ('Fig.11 / Leonardo bridge 6', 'bridge', ()),
    'fig12a': ('Fig.12(a) / two blocks — reconstructed', 'missing_a', ()),
    'fig12b': ('Fig.12(b) / two blocks — reconstructed', 'missing_b', ()),
    'fig12c': ('Fig.12(c) / Soma 3', 'datainfo7', ()),
    'fig12d': ('Fig.12(d) / unstable Soma 3', 'datainfo0209_3', ()),
    'fig12e': ('Fig.12(e) / Soma 4', 'datainfo4', ()),
    'fig12f': ('Fig.12(f) / Domino 3', 'domino_5', ()),
    'fig12g': ('Fig.12(g) / offset stack', 'domino_7', ()),
    'fig12h': ('Fig.12(h) / supported beam', 'domino_8', ()),
    'fig13a': ('Fig.13(a) / BL voxel representation', 'single', ()),
    'fig13b': ('Fig.13(b) / BL + Z legacy candidate', 'datainfo_ss', ('Z','bigL')),
    'fig13c_replacement': ('Fig.13(c) / seeded replacement, 5 blocks', 'random5', ()),
    'fig13d_replacement': ('Fig.13(d) / seeded replacement, 7 blocks', 'random7', ()),
    'fig15a_soma4': ('Fig.15(a) / four-Soma legacy candidate', 'datainfo4', ()),
    'fig15b_domino4': ('Fig.15(b) / four-Domino legacy candidate', 'domino_6', ()),
    'fig15c_burr6': ('Fig.15(c) / Burr 6', 'burrpuzzle', ('u','3u','2L right','2L left','2u','1')),
    'fig15d_bridge6': ('Fig.15(d) / reconstructed bridge', 'bridge', ()),
}


@lru_cache(maxsize=32)
def mesh(name, nominal):
    raw = read_mesh(ASSETS/'meshes'/f'{name}.stl', length_unit='mm')
    if nominal and name in ('bigL','smallL_2','Z','T','cross','crossleft'):
        # These are voxel blocks: preserve STL faces, restore the declared grid.
        vertices = np.rint((raw.vertices+EDGE/2)/EDGE)*EDGE-EDGE/2
        if np.max(np.linalg.norm(vertices-raw.vertices,axis=1)) > 1e-7:
            raise ValueError('STL does not match the documented 18.5 mm voxel grid')
        return _grid_boundary(MeshData(vertices,raw.faces))
    return raw


def _asset(key):
    if 'Domino' in key: return 'domino'
    return {'smallL 2':'smallL_2','Cross':'cross','Crossleft':'crossleft'}.get(key,key).replace(' ','_')


def _legacy(name, nominal):
    data = json.loads((ASSETS/'scenes'/f'{name}.json').read_text(encoding='utf-8'))
    soma = name.startswith('datainfo')
    parts, corrections = [], []
    for key, value in data.items():
        rotation = np.asarray(value['rotation'],dtype=float)
        translation = np.asarray(value['location'],dtype=float)*.001
        raw_tf = pose(translation, legacy_rotation(rotation))
        if nominal:
            step = np.pi/4 if soma else np.pi/180
            rounded = np.rint(rotation/step)*step
            rotation = np.where(abs(rotation-rounded)<6e-7,rounded,rotation)
            if name == 'domino_5' and abs(translation[0])>.01:
                theta = np.deg2rad(20)
                translation[0] = np.sign(translation[0])*(.01+.01*np.cos(theta)+.12*np.sin(theta))
                translation[2] = .01*np.sin(theta)
        geometry = mesh(_asset(key),nominal)
        tf = pose(translation,legacy_rotation(rotation))
        original = mesh(_asset(key),False)
        before = original.vertices@raw_tf[:3,:3].T+raw_tf[:3,3]
        nominal_vertices = (np.rint((original.vertices+EDGE/2)/EDGE)*EDGE-EDGE/2
                            if nominal and soma else original.vertices)
        after = nominal_vertices@tf[:3,:3].T+tf[:3,3]
        corrections.append(dict(part=key,max_world_vertex_change_m=float(np.linalg.norm(after-before,axis=1).max())))
        parts.append(Part(key,geometry,tf))
    note = ('名义几何：保留原始位移；恢复 18.5 mm 网格、导出角度，重建等价体素表面消除 T 形接缝。' if soma else
            '原始 STL；恢复接近整度的导出角度。Domino 3 另按 20° 几何恢复相切位置。') if nominal else '原始 STL 和 JSON 位姿；未修正旧数据的尺度及间隙。'
    return parts, dict(source=f'asp/data/{name}',reproduction='nominal_legacy' if nominal else 'raw_legacy',
                      note=note,corrections=corrections)


def voxel_mesh(cubes):
    """Boundary of a voxel union; internal faces are removed, cavities preserved."""
    occupied = {tuple(map(int,c)) for c in cubes}
    vertices, faces = [], []
    for cube in sorted(occupied):
        for axis in range(3):
            for sign in (-1,1):
                normal = np.zeros(3,dtype=int); normal[axis] = sign
                if tuple(np.asarray(cube)+normal) in occupied: continue
                tangent = np.eye(3)[(axis+1)%3]
                bitangent = np.cross(normal,tangent)
                center = np.asarray(cube)+normal/2
                quad = np.array([center+(u*tangent+v*bitangent)/2 for u,v in ((-1,-1),(1,-1),(1,1),(-1,1))])*EDGE
                start = len(vertices); vertices.extend(quad)
                faces.extend(((start,start+1,start+2),(start,start+2,start+3)))
    return MeshData(vertices,faces)


def _grid_boundary(geometry):
    """Recover this small axis-aligned grid solid, not a general STL repair."""
    from wrs.assembly.geometry.preprocess import prepare_mesh
    prepared = prepare_mesh(geometry)
    if not np.allclose(np.max(abs(prepared.normals),axis=1),1,atol=1e-10):
        raise ValueError('Grid reconstruction requires axis-aligned voxel faces')
    cleaned = prepared.mesh
    lo = np.ceil(cleaned.vertices.min(0)/EDGE).astype(int)
    hi = np.floor(cleaned.vertices.max(0)/EDGE).astype(int)
    ids = np.array(list(product(*(range(a,b+1) for a,b in zip(lo,hi)))))
    relative = cleaned.vertices[cleaned.faces][None,:,:,:]-ids[:,None,None,:]*EDGE
    a,b,c = relative.transpose(2,0,1,3)
    la,lb,lc = np.linalg.norm(relative,axis=3).transpose(2,0,1)
    dot = lambda u,v:np.einsum('...i,...i->...',u,v)
    numerator = dot(a,np.cross(b,c))
    denominator = la*lb*lc+dot(a,b)*lc+dot(b,c)*la+dot(c,a)*lb
    winding = np.arctan2(numerator,denominator).sum(1)/(2*np.pi)
    if not np.allclose(winding,np.rint(winding),atol=1e-8):
        raise ValueError('Ambiguous voxel occupancy; do not automatically close an arbitrary STL')
    return voxel_mesh(ids[abs(winding)>.5])


def _random_parts(count, seed):
    """Deterministic analogue of Fig.13; no claim of the paper's stability filter."""
    shapes = (
        ((0,0,0),(0,0,1),(0,0,2),(1,0,2)), # BL
        ((0,0,0),(0,0,1),(1,0,1)),          # SL
        ((0,0,0),(0,0,1),(1,0,1),(1,0,2)), # Z
        ((0,0,0),(1,0,0),(2,0,0),(1,0,1)), # T
        ((0,0,0),(1,0,0),(1,1,0),(1,1,1)),
    )
    rotations = []
    for axes in permutations(range(3)):
        for signs in product((-1,1),repeat=3):
            r = np.eye(3,dtype=int)[:,axes]*signs
            if round(np.linalg.det(r)) == 1: rotations.append(r)
    rng = np.random.default_rng(seed)
    occupied, parts, records = set(), [], []
    for i in range(count):
        shape = np.array(shapes[i%len(shapes)])@rotations[int(rng.integers(24))].T
        if occupied:
            neighbors = sorted({tuple(np.array(c)+n) for c in occupied for n in np.r_[np.eye(3,dtype=int),-np.eye(3,dtype=int)]}-occupied)
            candidates = []
            for target in neighbors:
                for anchor in shape:
                    shifted = shape+np.asarray(target)-anchor
                    cells = {tuple(c) for c in shifted}
                    if occupied & cells: continue
                    bounds = np.array(sorted(occupied|cells))
                    volume = int(np.prod(np.ptp(bounds,axis=0)+1))
                    candidates.append((volume,tuple(map(tuple,shifted))))
            if not candidates: raise RuntimeError('No connected voxel placement')
            best = min(c[0] for c in candidates)
            ties = sorted(set(c[1] for c in candidates if c[0]==best))
            shape = np.array(ties[int(rng.integers(len(ties)))])
        occupied.update(map(tuple,shape))
        records.append(shape.tolist()); parts.append(Part(f'block_{i+1}',voxel_mesh(shape)))
    return parts, dict(source='deterministic voxel generator inspired by Fig.13',reproduction='replacement',
        seed=seed,voxel_edge_m=EDGE,voxels=records,
        note='固定种子的替代测试，不是图中原始随机样本；只保证不重叠、面连通，未按论文筛选最终稳定性。')


def _bridge():
    data = json.loads((ASSETS/'scenes'/'bridge.json').read_text())
    theta = np.deg2rad(20); s,c = np.sin(theta),np.cos(theta)
    width = .03
    rhs = -width/2*(1+abs(np.cos(2*theta))+abs(np.sin(2*theta)))
    y,z = (rhs-width)/(2*s),(rhs+width)/(2*c)
    parts = []
    for i,(key,value) in enumerate(data.items()):
        rotation = np.deg2rad(np.rint(np.rad2deg(value['rotation'])))
        location = np.array(value['location'])*.001
        if i in (0,3): location[1:3] = (y if i==0 else -y,z)
        parts.append(Part(key,box((width,width,.3)),pose(location,legacy_rotation(rotation))))
    return parts, dict(source='asp/data/bridge + reconstructed square beams',reproduction='reconstructed',
        section_m=[width,width],length_m=.3,
        note='原 alframe.stl 缺失。采用 30×30×300 mm 实心梁和解析相切位置；不是原铝型材。AF 颜色编号尚未与原图逐一确认。')


def make_case(key, *, nominal=True):
    title, name, order = CATALOG[key]
    if name == 'bridge':
        if not nominal: raise FileNotFoundError('原始 alframe.stl 缺失；raw 模式不能重建')
        parts, meta = _bridge()
    elif name.startswith('missing'):
        if not nominal: raise FileNotFoundError('原始 datainfo0209 / datainfo0209_2 场景文件缺失')
        left = [(0,0,0),(0,0,1),(1,0,1)]
        if name=='missing_a': left += [(2,0,1)]
        right = [(2,0,0),(3,0,0),(3,0,1)]
        parts = [Part('left',voxel_mesh(left)),Part('right',voxel_mesh(right))]
        meta = dict(source='Fig.12 schematic reconstruction',reproduction='reconstructed',
                    note='原 JSON 缺失；按图重建两件搭接示意，未声称精确复现原几何、姿态或稳定性分数。')
    elif name.startswith('random'):
        if not nominal: raise FileNotFoundError('论文随机种子和逐次场景未提供；仅提供固定种子的替代数据')
        parts,meta = _random_parts(int(name[6:]),20210913+int(name[6:]))
    elif name == 'single':
        parts = [Part('bigL',mesh('bigL',nominal))]
        meta = dict(source='asp/objects/bigL.stl',reproduction='nominal_legacy' if nominal else 'raw_legacy',note='单个 BL，用于检查体素表示与地面接触。')
    else:
        parts, meta = _legacy(name,nominal)
    if key in ('fig13b','fig15a_soma4','fig15b_domino4'):
        meta['note'] += ' 对应旧场景作为图中变体候选；未核实每一处原图位姿。'
    ids = tuple(p.part_id for p in parts)
    if order and set(order)!=set(ids): raise ValueError((key,order,ids))
    order = order or ids
    vertices = np.vstack([p.geometry.vertices@p.assembled_tf[:3,:3].T+p.assembled_tf[:3,3] for p in parts])
    lo,hi = vertices.min(0),vertices.max(0)
    ground = Part('ground',box((hi[0]-lo[0]+.06,hi[1]-lo[1]+.06,.01)),
                  pose(((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]-.005)),fixed=True)
    meta.update(title=title,figure_key=key,order=order,mode='nominal' if nominal else 'raw',
                order_note='检查用前缀顺序；不代表已验证的装配路径或双臂执行计划。',
                ground_rule='fixed box top at minimum assembled vertex z',length_unit='m')
    return Assembly(tuple([ground,*parts]),provenance=meta)


def prefix_state(assembly, stage=0):
    order = assembly.provenance['order']
    if not 0 <= stage <= len(order): raise ValueError('stage must be 0 (full) or 1..part_count')
    present = set(order[:stage] if stage else order)|{'ground'}
    return AssemblyState({p.part_id:p.assembled_tf for p in assembly.parts if p.part_id in present})
