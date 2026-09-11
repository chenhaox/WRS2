"""Sequence scenes with explicit masses, friction and metre-scale geometry."""
from _stability_cases import make_case as stability_case, block, case_supports
from wrs.assembly import Assembly, Part, analyze_contacts, build_contact_graph
from wrs.assembly.primitives import cylinder, pose

DESCRIPTIONS = {
    'stack': '两层堆叠，2 个活动零件。',
    'bridge': '双桥墩、横梁和偏心载荷，4 个活动零件。',
    'floating': '悬空堆叠，两个有容量限制的辅助支撑。',
    'gantry': '两层框架：四下柱、平台、两上柱、横梁、载荷；9 个活动零件。',
    'sleeve': '有间隙的轴、套筒、环形垫片、两根定位销；5 个活动零件。',
}


def make_case(name):
    if name in ('stack','bridge','floating'): return stability_case(name)
    if name=='gantry':
        parts=[block('table',(.45,.35,.02),(0,0,-.01),fixed=True)]
        parts += [block(f'lower_post_{i}',(.035,.035,.12),(x,y,.06),.3)
                  for i,(x,y) in enumerate(((-.1,-.065),(-.1,.065),(.1,-.065),(.1,.065)))]
        parts += [block('platform',(.28,.2,.02),(0,0,.13),1.),
                  block('upper_left',(.035,.07,.1),(-.08,0,.19),.3),
                  block('upper_right',(.035,.07,.1),(.08,0,.19),.3),
                  block('top_beam',(.24,.1,.02),(0,0,.25),.6),
                  block('payload',(.06,.06,.06),(.015,0,.29),1.5)]
    elif name=='sleeve':
        parts=[block('table',(.3,.25,.02),(0,0,-.01),fixed=True)]
        for pid,r,h,z,inner,x in (('sleeve',.04,.08,.04,.025,0),
                                 ('shaft',.022,.16,.08,0,0),
                                 ('washer',.042,.01,.085,.024,0),
                                 ('pin_left',.008,.10,.05,0,-.075),
                                 ('pin_right',.008,.10,.05,0,.075)):
            parts.append(Part(pid,cylinder(r,h,sections=24,inner_radius=inner),pose((x,0,z)),
                              mass_kg=.2,com_local_m=(0,0,0),friction=.5))
    else: raise ValueError('Unknown sequence case: '+name)
    assembly=Assembly(tuple(parts)); state=assembly.initial_state()
    return assembly,state,build_contact_graph(assembly,state,analyze_contacts(assembly,state))
