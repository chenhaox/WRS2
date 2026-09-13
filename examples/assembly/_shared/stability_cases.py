"""Small physical scenes: metres, kilograms, explicit COM and friction."""
import numpy as np
from wrs.assembly import (Assembly, Part, ExternalWrench, LoadCase, StabilityConfig,
                          SupportCandidate, analyze_contacts, build_contact_graph)
from wrs.assembly.geometry.primitives import box, cylinder, pose


DESCRIPTIONS = {
    'stack': '两层堆叠：1 kg + 2 kg，桌面承担 29.43 N。',
    'floating': '悬空堆叠：无支撑不可行；增加两个各限 20 N 的辅助支撑。',
    'bridge': '双墩桥：两个自由桥墩、横梁和偏心载荷；检查载荷传递和侧推。',
    'overhang': '偏载倾覆：横梁和载荷的合成质心越过右墩支撑边界。',
    'incline': '20° 斜面，摩擦系数 0.6：可保持平衡。',
    'slippery': '同一 20° 斜面，摩擦系数 0.2：不足以平衡重力切向分量。',
    'tripod': '三点支撑平台承载圆柱：三个分离接触区，检查扭矩扰动。',
}


def block(name, size, center, mass=1., *, fixed=False, friction=.5, rotation=None):
    return Part(name, box(size), pose(center, rotation), mass_kg=mass,
                com_local_m=(0,0,0), friction=friction, fixed=fixed)


def make_case(name):
    if name not in DESCRIPTIONS:
        raise ValueError(f'Unknown stability case: {name}')
    if name in ('stack','floating'):
        parts = [block('lower', (.1,.1,.1), (0,0,.05)),
                 block('upper', (.1,.1,.1), (0,0,.15), 2.)]
        if name == 'stack':
            parts.append(block('table', (.3,.3,.02), (0,0,-.01), fixed=True))
    elif name in ('bridge','overhang'):
        beam_x, payload_x = (0.,.06) if name == 'bridge' else (.06,.19)
        parts = [block('table', (.55,.32,.02), (0,0,-.01), fixed=True),
                 block('left_pier', (.04,.10,.10), (-.09,0,.05)),
                 block('right_pier', (.04,.10,.10), (.09,0,.05)),
                 block('beam', (.34,.12,.03), (beam_x,0,.115)),
                 block('payload', (.06,.06,.06), (payload_x,0,.16), 3.)]
    elif name in ('incline','slippery'):
        angle = np.deg2rad(20)
        c,s = np.cos(angle), np.sin(angle)
        R = np.array([[c,0,s],[0,1,0],[-s,0,c]])
        mu = .6 if name == 'incline' else .2
        parts = [block('slope', (.3,.22,.02), R @ [0,0,-.01], fixed=True, friction=mu, rotation=R),
                 block('slider', (.10,.10,.04), R @ [0,0,.02], 2., friction=mu, rotation=R)]
    else:
        parts = [block(f'foot_{i}', (.025,.025,.10), (x,y,.05), fixed=True)
                 for i,(x,y) in enumerate(((-.09,-.06),(.09,-.06),(0,.09)))]
        parts += [block('platform', (.24,.24,.02), (0,0,.11)),
                  Part('payload', cylinder(.035,.08,sections=24), pose((.02,0,.16)),
                       mass_kg=2., com_local_m=(0,0,0), friction=.5)]
    assembly = Assembly(tuple(parts))
    state = assembly.initial_state()
    graph = build_contact_graph(assembly,state,analyze_contacts(assembly,state))
    return assembly, state, graph


def case_config(name):
    if name in ('bridge','overhang'):
        loads = (LoadCase('push_1N', (ExternalWrench('payload',(1,0,0)),)),
                 LoadCase('push_30N', (ExternalWrench('payload',(30,0,0)),)))
    elif name == 'tripod':
        loads = (LoadCase('twist_0.1Nm', (ExternalWrench('payload',torque_world_nm=(0,0,.1)),)),
                 LoadCase('twist_1Nm', (ExternalWrench('payload',torque_world_nm=(0,0,1.)),)))
    else:
        loads = ()
    return StabilityConfig(disturbances=loads)


def case_supports(name):
    if name == 'floating':
        return (SupportCandidate('left','lower',(-.03,0,0),(0,0,1),20.),
                SupportCandidate('right','lower',(.03,0,0),(0,0,1),20.))
    return ()
