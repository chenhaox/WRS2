"""WRS static equilibrium: switch scenes, bodies, load cases and force overlays."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from wrs.assembly import check_equilibrium, find_support_requirements
from _stability_cases import DESCRIPTIONS, make_case, case_config, case_supports
from _contact_display import contact_layers, draw_contacts


def solve_case(name):
    assembly,state,graph = make_case(name)
    config = case_config(name)
    result = check_equilibrium(assembly,state,graph,config=config)
    print(f'{name}: without auxiliary supports = {result.status}',flush=True)
    candidates = case_supports(name)
    if candidates:
        search = find_support_requirements(assembly,state,graph,candidates,config=config)
        print('Finite support requirements:',search.status,search.selected_support_ids,flush=True)
        if search.status == 'requirements_found':
            result = search.equilibrium
    print(f'  equilibrium={result.status}, {result.diagnostics["elapsed_s"]*1000:.3f} ms, '
          f'{len(result.force_sites)} sites; disturbances='
          f'{[(c.name,c.status) for c in result.disturbances]}',flush=True)
    return assembly,state,graph,config,result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=tuple(DESCRIPTIONS),default='bridge')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--all',action='store_true',help='check all scenes without opening a viewer')
    parser.add_argument('--port',type=int,default=8893)
    args = parser.parse_args()
    if args.all or args.headless:
        for name in DESCRIPTIONS if args.all else (args.case,):
            solve_case(name)
        return

    from wrs import wvw,wssop
    from wrs.viewer.web_ui import Anchor
    from _stability_display import draw_stability
    base = wvw.World(cam_pos=(.48,-.65,.45),cam_lookat_pos=(0,0,.09),port=args.port)
    loaded = False

    def show_case(name):
        nonlocal loaded
        assembly,state,graph,config,result = solve_case(name)
        for obj in tuple(base.scene):
            base.scene.remove(obj)
        if loaded:
            base.ui.remove_panel('contacts')
            base.ui.remove_panel('stability')
        models = []
        for part in assembly.parts:
            model = wssop.mesh(part.geometry.vertices,part.geometry.faces,
                              rgb=(.55,.67,.78) if part.fixed else (.83,.69,.37))
            model.tf = state.poses[part.part_id]
            base.scene.add(model)
            models.append(model)
        layers = contact_layers(p for edge in graph.edges for p in edge.patches)
        draw_contacts(base,layers,models,description=DESCRIPTIONS[name])
        draw_stability(base,assembly,state,result,config)
        base.set_caption(f'Static equilibrium | {name} | {result.status}')
        loaded = True

    panel = base.ui.add_panel('scenario',title='稳定性案例',anchor=Anchor.BOTTOM_LEFT,
                              width=250,offset=16,font_size=12)
    panel.add_select('case',label='切换案例',options=list(DESCRIPTIONS),value=args.case,on_change=show_case)
    show_case(args.case)
    base.run()


if __name__ == '__main__':
    main()
