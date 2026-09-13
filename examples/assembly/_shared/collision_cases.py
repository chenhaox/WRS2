"""Primitive and original STL geometry for SDF collision checks; no temporary STL files."""
from pathlib import Path
import numpy as np
from wrs.assembly import ContactModel, MeshData
from wrs.assembly.primitives import box, sphere, cylinder, pose
ROOT = Path(__file__).resolve().parents[3]
STLS = {'bunny': 'bunny.stl', 'flange': 'link6.stl', 'cylinder_stl': 'examples/l1picking/cylinder.stl',
        'gripper_finger': 'wrs/robots/end_effectors/openarm_gripper/meshes/finger.stl',
        'gripper_hand': 'wrs/robots/end_effectors/openarm_gripper/meshes/hand.stl',
        'ur3_forearm': 'wrs/robots/manipulators/universal_robots/ur3/meshes/forearm.stl',
        'fr3_link7': 'wrs/robots/manipulators/franka/fr3/meshes/link7.stl',
        'xarm_link3': 'wrs/robots/manipulators/xarm/lite6/meshes/link3.stl'}

def torus():
    u, v = np.meshgrid(np.arange(48)*2*np.pi/48, np.arange(16)*2*np.pi/16, indexing='ij')
    vertices = np.stack(((.02+.006*np.cos(v))*np.cos(u), (.02+.006*np.cos(v))*np.sin(u), .006*np.sin(v)), axis=-1)
    faces = []
    for i in range(48):
        for j in range(16):
            a, b, c, d = i*16+j, ((i+1)%48)*16+j, ((i+1)%48)*16+(j+1)%16, i*16+(j+1)%16
            faces.extend(((a, b, c), (a, c, d)))
    return MeshData(vertices.reshape(-1, 3), faces)

def concave_l():
    xy = np.array([[0, 0], [3, 0], [3, 1], [1, 1], [1, 3], [0, 3]])*.01
    vertices = np.concatenate([np.column_stack((xy, np.full(6, z))) for z in (-.005, .005)])
    faces = []
    for i in range(1, 5):
        faces.extend(((0, i+1, i), (6, 6+i, 7+i)))
    for i in range(6):
        j = (i+1)%6
        faces.extend(((i, j, j+6), (i, j+6, i+6)))
    return MeshData(vertices, faces)


def make_case(name, state):
    if state not in ('gap','touch','penetration'): raise ValueError(state)
    expected = {'gap':'separated','touch':'touching','penetration':'penetrating'}[state]
    if name == 'shaft':
        a = ContactModel(cylinder(.015,.02,32,inner_radius=.01),'A')
        b = ContactModel(cylinder(.01 if state=='touch' else .0098,.01,32),'B',
                         pose((.002 if state=='penetration' else 0,0,0)))
        return (a,b), expected
    primitives = {'box':lambda:box((.03,.025,.02)), 'sphere':lambda:sphere(.02),
        'cylinder':lambda:cylinder(.012,.04), 'tube':lambda:cylinder(.02,.02,inner_radius=.01),
        'torus':torus, 'concave_l':concave_l}
    b = (ContactModel(primitives[name](),'B') if name in primitives else
         ContactModel.from_file(ROOT/STLS[name],name='B',length_unit='m'))
    lo,hi = b.geometry.vertices.min(0),b.geometry.vertices.max(0)
    a = ContactModel(box((*((hi-lo)[:2]+.02),.02)),'A',pose((*(lo+hi)[:2]/2,-.01)))
    z = {'gap':.002,'touch':0.,'penetration':-.002}[state]
    return (a,b.at(pose((0,0,z-lo[2])))), expected
