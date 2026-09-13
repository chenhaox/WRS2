"""Small WRS viewer shared by contact and collision examples."""
import numpy as np
from wrs import wvw
from wrs.assembly.adapters.wrs_scene import scene_object_from_part
from .contact_display import draw_contacts


def show(assembly, layers, *, description, port):
    state = assembly.initial_state()
    vertices = np.vstack([
        p.geometry.vertices @ state.poses[p.part_id][:3, :3].T + state.poses[p.part_id][:3, 3]
        for p in assembly.parts
    ])
    center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
    span = max(float(np.ptp(vertices, axis=0).max()), .01)
    base = wvw.World(cam_pos=center + span * np.array([1.4, -1.8, 1.4]),
                     cam_lookat_pos=center, port=port)
    models = []
    for part in assembly.parts:
        model = scene_object_from_part(part, collision=False)
        model.tf = state.poses[part.part_id]
        base.scene.add(model)
        models.append(model)
    # Adaptive cells may contain T-junctions; their edges are not region outlines.
    draw_contacts(base, layers, models, description=description, draw_boundaries=False)
    base.run()
