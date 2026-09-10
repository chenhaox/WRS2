"""Evidence-only HTML previews and optional WRS scenes; no contact decisions."""
import json
from pathlib import Path
import numpy as np
from .model import to_dict

COLORS = {'active': '#078b70', 'near': '#e89420', 'interference': '#d74850', 'unknown': '#8466b4'}


def preview_case(name, description, assembly, state, analysis):
    """Make a JSON-compatible preview, with meshes at explicit world poses."""
    return {'name': name, 'description': description,
            'meshes': [{'id': part.part_id,
                        'vertices': part.geometry.vertices @ state.poses[part.part_id][:3,:3].T
                                    + state.poses[part.part_id][:3,3],
                        'faces': part.geometry.faces}
                       for part in assembly.parts if part.part_id in state.poses],
            'analysis': analysis.to_dict()}


def write_contact_html(cases, path):
    """Write a standalone orbitable preview, without network libraries/services.

    Parameters
    ----------
    cases : sequence of dict
        Records returned by ``preview_case``; units remain metres in the data.
    path : path-like
        Output HTML. Camera controls affect display only, never analysis.
    """
    payload = json.dumps(to_dict(cases), ensure_ascii=False, allow_nan=False).replace('<', '\\u003c')
    template = Path(__file__).with_name('_contact_viewer.html').read_text(encoding='utf-8')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(template.replace('__ASSEMBLY_DATA__', payload), encoding='utf-8')


def build_wrs_scene(assembly, state, analysis):
    """Build a WRS scene lazily, with original meshes and evidence overlays.

    This optional bridge creates render models only. It does not use collider
    proxies, start the viewer or modify assembly state. Call World.run yourself.
    """
    from wrs import wss, wssop
    scene = wss.Scene()
    for i, part in enumerate(assembly.parts):
        if part.part_id not in state.poses:
            continue
        obj = wssop.mesh(part.geometry.vertices, part.geometry.faces,
                         rgb=(.35,.52,.7) if i % 2 else (.68,.73,.8), alpha=.24, name=part.part_id)
        obj.tf = state.poses[part.part_id]
        scene.add(obj)
    for k, patch in enumerate(analysis.patches):
        color = COLORS[patch.classification]
        rgb = tuple(int(color[i:i+2],16)/255 for i in (1,3,5))
        vs, fs = [], []
        for region in patch.regions:
            for cell in region.cells_world_m:
                if patch.dimension == 2:
                    start = len(vs)
                    vs.extend(cell)
                    fs.extend([[start,start+i,start+i+1] for i in range(1,len(cell)-1)])
                elif len(cell) == 2:
                    scene.add(wssop.cylinder(cell[0],cell[1],radius=.00025,rgb=rgb))
        if fs:
            scene.add(wssop.mesh(vs,fs,rgb=rgb,alpha=.85,name=f'contact_{k}'))
        stride = max(1,len(patch.points_a_world_m)//12)
        for p, n in zip(patch.points_a_world_m[::stride],patch.normals_a_world[::stride]):
            scene.add(wssop.arrow(p,p+n*.008,shaft_radius=.00015,head_radius=.0004,rgb=rgb))
    return scene
