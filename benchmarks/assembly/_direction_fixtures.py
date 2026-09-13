"""Stable direction fixtures for comparing complete production API costs."""
import numpy as np
from wrs.assembly import Assembly, Part, analyze_contacts, build_contact_graph, contact_constraints
from wrs.assembly.primitives import box, cylinder, pose
from examples.assembly._shared.contact_display import contact_layers


def make_case(name):
    """Boxes use actual contact extraction; shaft uses an explicit ideal model."""
    part = Part('part', box())
    floor = Part('floor', box((.18,.18,.02)), pose((0,0,-.06)), fixed=True)
    ceiling = Part('ceiling', box((.18,.18,.02)), pose((0,0,.06)), fixed=True)
    left = Part('left', box((.02,.1,.1)), pose((-.06,0,0)), fixed=True)
    back = Part('back', box((.1,.02,.1)), pose((0,-.06,0)), fixed=True)
    if name in ('shaft', 'blind_shaft'):
        # Exact zero-clearance cylindrical model, not an inferred SDF contact.
        # Four radial inequalities are sufficient for its translation cone.
        parts = (Part('part', cylinder(.025,.09,64)),
                 Part('tube', cylinder(.045,.12,64,inner_radius=.025), fixed=True))
        normals = np.array([[1.,0,0],[-1,0,0],[0,1,0],[0,-1,0]])
        if name == 'blind_shaft':
            parts += (Part('bottom', cylinder(.025,.01,64), pose((0,0,-.05)), fixed=True),)
            normals = np.vstack((normals, [0,0,1]))
        assembly = Assembly(parts)
        # Display the declared mating region independently of contact inference.
        triangles = parts[0].geometry.vertices[parts[0].geometry.faces]
        dz = np.ptp(triangles[:,:,2],axis=1)
        side = triangles[dz > 0]
        layers = [dict(name='shaft / tube',kind='declared',dimension=2,
                       cells=list(side),points=np.empty((0,3)))]
        if name == 'blind_shaft':
            bottom = triangles[np.all(triangles[:,:,2] == -.045,axis=1)]
            layers.append(dict(name='shaft / bottom',kind='declared',dimension=2,
                               cells=list(bottom),points=np.empty((0,3))))
        return assembly, assembly.initial_state(), normals, layers
    parts = {'plane': (floor,part), 'channel': (floor,ceiling,part),
             'corner': (floor,left,back,part),
             'blocked': (floor,ceiling,left,back,part,
                         Part('right', box((.02,.1,.1)), pose((.06,0,0)), fixed=True),
                         Part('front', box((.1,.02,.1)), pose((0,.06,0)), fixed=True))}
    if name not in parts:
        raise ValueError(f'Unknown example: {name}')
    assembly = Assembly(parts[name])
    state = assembly.initial_state()
    analysis = analyze_contacts(assembly, state)
    graph = build_contact_graph(assembly, state, analysis)
    rows = contact_constraints(graph, ['part'])
    if rows.issues:
        raise RuntimeError(f'Example contacts need review: {rows.issues}')
    layers = contact_layers(p for p in analysis.patches if 'part' in (p.part_a,p.part_b))
    return assembly, state, rows.matrix_world[:, :3], layers
