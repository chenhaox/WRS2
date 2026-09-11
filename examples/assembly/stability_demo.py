"""WRS: two stacked blocks, with a table or two finite auxiliary supports."""
import argparse
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from wrs.assembly import (Assembly, Part, SupportCandidate, analyze_contacts,
                          build_contact_graph, check_equilibrium, find_support_requirements)
from wrs.assembly.primitives import box, pose
from _contact_display import contact_layers, draw_contacts


def make_case(name):
    # Mass (kg), local COM (m) and friction are explicit inputs.
    parts = [Part('lower', box(), pose((0,0,.05)), mass_kg=1., com_local_m=(0,0,0), friction=.5),
             Part('upper', box(), pose((0,0,.15)), mass_kg=2., com_local_m=(0,0,0), friction=.5)]
    if name == 'stack':
        parts.append(Part('table', box((.3,.3,.02)), pose((0,0,-.01)), fixed=True, friction=.5))
    assembly = Assembly(tuple(parts))
    state = assembly.initial_state()
    graph = build_contact_graph(assembly, state, analyze_contacts(assembly, state))
    return assembly, state, graph


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('stack','floating'), default='stack')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    assembly, state, graph = make_case(args.case)
    result = check_equilibrium(assembly, state, graph)
    print('Without auxiliary support:', result.status)

    if args.case == 'floating':
        # Each support supplies at most 20 N upward; neither alone carries 29.43 N.
        candidates = [SupportCandidate('left', 'lower', (-.03,0,0), (0,0,1), 20.),
                      SupportCandidate('right', 'lower', (.03,0,0), (0,0,1), 20.)]
        search = find_support_requirements(assembly, state, graph, candidates)
        print('Support requirements:', search.status, search.selected_support_ids)
        result = search.equilibrium
        print('Support locations are declared; robot execution is not verified.')

    print(f'Equilibrium: {result.status}, {result.diagnostics["elapsed_s"]*1000:.3f} ms')
    print('Residuals:', result.nominal.body_residuals)
    if args.headless:
        return

    from wrs import wvw, wssop
    base = wvw.World(cam_pos=(.42,-.58,.4), cam_lookat_pos=(0,0,.08), port=8893)
    base.set_caption(f'Static equilibrium | {args.case} | red gravity, green reaction | 0.004 m/N')
    part_models = []
    for part in assembly.parts:
        model = wssop.mesh(part.geometry.vertices, part.geometry.faces,
                           rgb=(.55,.67,.78) if part.fixed else (.83,.69,.37), alpha=.4)
        model.tf = state.poses[part.part_id]
        model.add_to_scene(base.scene)
        part_models.append(model)
        if not part.fixed:
            com = model.tf[:3,:3] @ part.com_local_m+model.tf[:3,3]
            gravity = part.mass_kg*assembly.gravity_world_m_s2
            wssop.arrow(com, com+.004*gravity, shaft_radius=.001,
                         head_radius=.003, head_length=.01, rgb=(.85,.2,.15)).add_to_scene(base.scene)
    layers = contact_layers(p for edge in graph.edges for p in edge.patches)
    draw_contacts(base,layers,part_models,
                  description='绿色面：接触区域。红箭头：重力。绿箭头：接触力。')
    fixed_ids = {part.part_id for part in assembly.parts if part.fixed}
    for force in (*result.nominal.contact_forces, *result.nominal.support_forces):
        # Show the free body's reaction. For two free bodies, show B only.
        f = force['force_on_a_world_n'] if force['part_b'] in fixed_ids else force['force_on_b_world_n']
        if np.linalg.norm(f) > 1e-8:
            p = force['point_world_m']
            wssop.arrow(p, p+.004*f, shaft_radius=.001,
                         head_radius=.003, head_length=.01, rgb=(.05,.65,.4)).add_to_scene(base.scene)
    base.run()


if __name__ == '__main__':
    main()
