"""Shared geometry/drawing for the two short WRS direction examples."""
import numpy as np
from wrs.assembly import (Assembly, Part, analyze_contacts, build_contact_graph,
                          contact_constraints, fibonacci_directions)
from wrs.assembly.primitives import box, cylinder, pose
from _contact_display import contact_layers, draw_contacts


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


def draw_result(base, assembly, state, normals, result, contacts):
    """Left: assembly; right: direction sphere. All arrows are computed results."""
    from wrs import wssop
    part_offset = np.array([-.14,0,0])
    center = np.array([.14,0,0])
    radius = .085
    part_models = []
    for part in assembly.parts:
        color = (.36,.54,.70) if part.fixed else (.83,.69,.37)
        model = wssop.mesh(part.geometry.vertices, part.geometry.faces,
                           rgb=color, alpha=.28 if part.fixed else .6, name=part.part_id)
        tf = state.poses[part.part_id].copy()
        tf[:3,3] += part_offset
        model.tf = tf
        model.add_to_scene(base.scene)
        part_models.append(model)
    draw_contacts(base,contacts,part_models,offset=part_offset,
                  description='左：装配与接触面。右：方向球。蓝箭头为选中方向。')
    # Gray points only outline the sphere. They are not solver candidates.
    outline = fibonacci_directions(650)
    wssop.point_cloud(center+radius*outline, np.tile((.68,.72,.77),(len(outline),1)),
                      alpha=.28).add_to_scene(base.scene)
    unique = np.unique(normals/np.linalg.norm(normals,axis=1)[:,None],axis=0)
    for n in unique:
        wssop.arrow(center, center+.065*n, shaft_radius=.0006,
                     head_radius=.002, head_length=.006, rgb=(.83,.22,.18)).add_to_scene(base.scene)
    if result.method == 'fibonacci' and len(result.directions):
        points = center+radius*result.directions
        if len(points) > 2:
            wssop.point_cloud(points, np.tile((.04,.65,.43),(len(points),1))).add_to_scene(base.scene)
        else:
            for p in points:
                wssop.sphere(pos=p, radius=.004, rgb=(.04,.65,.43)).add_to_scene(base.scene)
    if result.best_direction is not None:
        for origin, length in ((center,radius), (part_offset,.13)):
            wssop.arrow(origin, origin+length*result.best_direction, shaft_radius=.0015,
                         head_radius=.004, head_length=.012, rgb=(.1,.35,.95)).add_to_scene(base.scene)
    wssop.frame(pos=part_offset+[-.03,-.1,-.07], length_scale=.18, alpha=.5).add_to_scene(base.scene)


def print_result(case, result):
    print(f'{case} / {result.method}: {result.status}, dimension={result.dimension}, '
          f'directions={len(result.directions)}, {result.diagnostics["elapsed_s"]*1000:.3f} ms')
    print('best direction:', result.best_direction)
    print('Blue: chosen direction. Red: constraint normals. Green: feasible samples. Gray: sphere outline.')
    if case in ('shaft','blind_shaft'):
        print('Shaft uses declared ideal zero-clearance cylindrical constraints; mesh is for display.')
