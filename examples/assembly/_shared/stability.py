"""WRS static equilibrium: switch scenes, bodies, load cases and force overlays."""
from dataclasses import replace

from wrs.assembly import check_equilibrium, find_support_requirements
from .stability_cases import DESCRIPTIONS, make_case, case_config, case_supports
from .contact_display import contact_layers, draw_contacts


def solve_case(name,*,reduce_contact_points=True,with_supports=True,prepared=None,curved_point_budget=64):
    assembly,state,graph = make_case(name) if prepared is None else prepared
    config = replace(case_config(name),reduce_contact_points=reduce_contact_points,
                     curved_point_budget=curved_point_budget)
    result = check_equilibrium(assembly,state,graph,config=config)
    print(f'{name}: without auxiliary supports = {result.status}',flush=True)
    candidates = case_supports(name) if with_supports else ()
    if candidates:
        search = find_support_requirements(assembly,state,graph,candidates,config=config)
        print('Finite support requirements:',search.status,search.selected_support_ids,flush=True)
        if search.status == 'requirements_found':
            result = search.equilibrium
    print(f'  equilibrium={result.status}, {result.diagnostics["elapsed_s"]*1000:.3f} ms, '
          f'{len(result.force_sites)} sites; disturbances='
          f'{[(c.name,c.status) for c in result.disturbances]}',flush=True)
    return assembly,state,graph,config,result


def show(case="bridge", *, port=8893, reduce_contact_points=True, aux_supports=True, curved_point_budget=64):
    from wrs import wvw,wssop
    from wrs.viewer.web_ui import Anchor
    from .stability_display import draw_stability
    base = wvw.World(cam_pos=(.48,-.65,.45),cam_lookat_pos=(0,0,.09),port=port)
    loaded = False
    prepared_cases = {}
    settings = dict(case=case,reduce=reduce_contact_points,supports=aux_supports)

    def show_case(name):
        nonlocal loaded
        settings['case'] = name
        if name not in prepared_cases:
            prepared_cases[name] = make_case(name)
        assembly,state,graph,config,result = solve_case(name,prepared=prepared_cases[name],
            reduce_contact_points=settings['reduce'],with_supports=settings['supports'],
            curved_point_budget=curved_point_budget)
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
    panel.add_select('case',label='切换案例',options=list(DESCRIPTIONS),value=case,on_change=show_case)
    def change_option(key,value):
        settings[key] = value=='开启'
        show_case(settings['case'])
    panel.add_select('reduce',label='约简力点',options=['开启','关闭'],
        value='开启' if settings['reduce'] else '关闭',on_change=lambda v:change_option('reduce',v))
    panel.add_label('clean',label='清理规则',value='重复点始终清理；关闭约简可保留其余原始力点。')
    panel.add_select('supports',label='floating 辅助支撑',options=['开启','关闭'],
        value='开启' if settings['supports'] else '关闭',on_change=lambda v:change_option('supports',v))
    show_case(case)
    base.run()
