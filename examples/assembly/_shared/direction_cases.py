"""Nine contact-cone cases constructed from real box/fixture meshes."""
import numpy as np
from wrs.assembly import Assembly, Part
from wrs.assembly.primitives import box, pose

# Rows describe fixture placement only. The solver receives normals extracted
# from actual contact regions, independently of this expected-value table.
CASES = {
    'a': ('单面 / 半球', [(0,0,1)], 10, 3),
    'b': ('相反双面 / 完整大圆', [(1,0,0),(-1,0,0)], 9, 2),
    'c': ('两面转角 / 球面楔形', [(1,0,0),(0,1,0)], 10, 3),
    'd': ('平槽加挡板 / 半圆', [(1,0,0),(-1,0,0),(0,1,0)], 3, 2),
    'e': ('四面导向 / 双向轴', [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0)], 2, 1),
    'f': ('三面转角 / 球面区域', [(1,0,0),(0,1,0),(0,0,1)], 10, 3),
    'g': ('平槽加两挡板 / 圆弧', [(1,0,0),(-1,0,0),(0,1,0),(0,0,1)], 3, 2),
    'h': ('五面盲槽 / 单向轴', [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1)], 1, 1),
    'i': ('六面包围 / 平移锁死', [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)], 0, 0),
}


def make_case(key):
    title, rows, score, dimension = CASES[key]
    parts = [Part('moving', box((.06,.06,.06)))]
    for i,row in enumerate(rows):
        n = np.asarray(row,dtype=float)
        size = np.full(3,.05); size[np.argmax(abs(n))] = .01
        parts.append(Part(f'fixture_{i}',box(size),pose(-.035*n),fixed=True))
    return Assembly(tuple(parts),provenance=dict(case=key,title=title,source='constructed_mesh_fixture',
        expected_score=score,expected_dimension=dimension,expected_active_area_m2=len(rows)*.05**2))
