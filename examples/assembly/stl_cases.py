"""Real, unmodified WRS STL fixtures with explicitly declared metre units."""
from pathlib import Path
import numpy as np
from wrs.assembly import ContactModel, ContactConfig
from wrs.assembly.primitives import box, pose

ROOT = Path(__file__).resolve().parents[2]
FILES = {'bunny': ('bunny.stl', 'Bunny / 170 mm'),
         'bunny_small': ('bunny_small.stl', 'Bunny / 48 mm'),
         'flange': ('link6.stl', '法兰 STL'),
         'stl_cylinder': ('examples/l1picking/cylinder.stl', '圆柱 STL')}


def make_case(key):
    """Place the actual STL 0.2 mm above a closed support, without remeshing."""
    filename, label = FILES[key]
    model = ContactModel.from_file(ROOT/filename, name='B', length_unit='m')
    lo, hi = model.geometry.vertices.min(0), model.geometry.vertices.max(0)
    model = model.at(pose((0, 0, .0002-lo[2])))
    support = ContactModel(box((*((hi-lo)[:2]+.02), .006)), 'A',
                           pose((*(lo+hi)[:2]/2, -.003)))
    description = (f'原始 {filename}，{len(model.geometry.faces)} 个三角形，单位 m；'
                   '模型底部与支撑正间隙 0.2 mm，显示 0.5 mm 近接触带。')
    if key.startswith('bunny'):
        description += '该文件检测到一对自交三角形：bunny 作为目标时只查询无符号距离，不能认证实体内部。'
    return label, description, (support, model), ContactConfig(
        near_tol_m=.0005, surface_resolution_m=.001, normal_angle_rad=.55,
        max_cells=60000, max_triangle_tests=500000)
