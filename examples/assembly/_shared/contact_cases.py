"""Primitive, STL and supplied-grid contact scenes, in declared SI units."""
from dataclasses import replace
import numpy as np
from wrs.assembly import Assembly, Part, ContactConfig, ContactModel, GridSDF
from wrs.assembly.geometry.primitives import box, rectangle, rectangular_ring, cylinder, sphere, pose
from wrs.assembly.geometry.transforms import rotation_xyz
from .stl_cases import FILES, make_case as stl_case

def cases():
    """Return independent deterministic cases; dimensions and expected areas in SI."""
    def pair(a,b,tf=None):
        return Assembly((Part('A',a,fixed=True),Part('B',b,pose() if tf is None else tf)))
    default = ContactConfig()
    curve = ContactConfig(near_tol_m=.0005, surface_resolution_m=.003,
                          normal_angle_rad=.55,max_cells=4000,max_triangle_tests=30000)
    shaft_config = ContactConfig(near_tol_m=.0005, surface_resolution_m=.003,
                                 normal_angle_rad=.55,max_cells=4000,max_triangle_tests=100000)
    return {
        'boxes': ('箱体的部分面接触', '两个 100 mm 箱体横向错开 25 mm。解析接触面积为 7,500 mm²。',
                  pair(box(),box(),pose((.025,0,.1))), default, .0075),
        'ring': ('带孔接触面', '100 mm 方环，中孔 40 mm；接触面积 8,400 mm²。俯视可检查孔洞和完整边界。',
                 pair(rectangular_ring(),rectangle(upward=False)), default, .0084),
        'gap': ('0.5 μm 正间隙', '虽然间隙很小，默认仍是 near，active 面积为零；没有静默吸附。',
                pair(box(),box(),pose((0,0,.1000005))), default, 0),
        'tilted': ('倾斜面的线接触', '绕 Y 轴旋转 0.02 rad；间隙沿平面变化。零间隙线、正间隙带和局部干涉分别记录。',
                   pair(rectangle(),rectangle(upward=False),pose(rotation=rotation_xyz([0,.02,0]))), default, 0),
        'penetration': ('独立的穿透诊断', '两个箱体重叠 10 mm。实体诊断必须报告 penetrating，不能仅依据局部接触区域判断。',
                        pair(box(),box(),pose((0,0,.09))), default, 0),
        'sphere': ('球面与平面切触', '半径 25 mm 的离散球面与平面切触。显示的是 0.5 mm 近接触带估计，不能当成有限承载面积。',
                   pair(rectangle(),sphere(sections=16,rings=8),pose((0,0,.025))),curve,0),
        'shaft': ('轴孔的正间隙', '轴半径 9.8 mm、孔半径 10 mm；径向设计间隙 0.2 mm。橙色是 0.5 mm 距离阈值内的近接触带，实际接触面积为零。A 是孔壁，B 是轴壁；孔壁端部的带边界仍有分辨率误差。',
                  pair(cylinder(radius=.015,inner_radius=.01,height=.02,sections=32),
                       cylinder(radius=.0098,height=.01,sections=32)),shaft_config,0),
    }

def grid_case():
    """Two boxes represented by sampled analytical SDFs; x,y,z array ordering."""
    axis = np.linspace(-.008, .008, 81)
    xyz = np.stack(np.meshgrid(axis, axis, axis, indexing='ij'), axis=-1)
    q = np.abs(xyz)-.002
    values = np.linalg.norm(np.maximum(q, 0), axis=-1)+np.minimum(q.max(axis=-1), 0)
    spacing = float(axis[1]-axis[0])
    # For exact samples of a 1-Lipschitz SDF, trilinear interpolation error is
    # <= sqrt(sum(h_i**2))/2 by convex weights and the weighted variance bound.
    grid = GridSDF(values, [axis[0]]*3, spacing, error_bound_m=np.sqrt(3)*spacing/2)
    mesh = box((.004, .004, .004))
    a = ContactModel(mesh, 'A', representations={'sdf': grid})
    b = ContactModel(mesh, 'B', pose((0, 0, .005)), {'sdf': grid})
    return ('传入体素 SDF：箱体间隙',
            '两个 4 mm 箱体相距 1 mm。SDF 数组步长 0.2 mm，声明插值误差上界约 0.173 mm；'
            '面积在输入网格上积分，网格场的全局实体关系保留 unknown。',
            (a, b), ContactConfig(near_tol_m=.002, surface_resolution_m=.0002,
                                  normal_angle_rad=.55, max_cells=20000))


def make_case(name):
    if name == 'grid': return grid_case()
    if name in FILES: return stl_case(name)
    title, description, assembly, config, expected = cases()[name]
    if name == 'shaft':
        config = replace(config, surface_resolution_m=.001, max_cells=60000, max_triangle_tests=500000)
    return title, description, tuple(ContactModel.from_part(p) for p in assembly.parts), config
