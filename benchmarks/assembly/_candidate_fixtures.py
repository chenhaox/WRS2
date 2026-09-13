"""Original local-cone fixtures retained for comparable historical measurements."""
from dataclasses import replace
from wrs.assembly import Assembly, Part, MatingRelation, ConstraintConfig, SDFContactBackend, SDFConfig
from wrs.assembly.primitives import box, cylinder, pose


def fixtures():
    """Return actual geometry fixtures and explicit moving IDs/backend controls."""
    part = Part('part', box())
    base = Part('floor', box((.18, .18, .02)), pose((0, 0, -.06)), fixed=True)
    top = replace(base, part_id='ceiling', assembled_tf=pose((0, 0, .06)))
    left = Part('left', box((.02, .1, .1)), pose((-.06, 0, 0)), fixed=True)
    back = Part('back', box((.1, .02, .1)), pose((0, -.06, 0)), fixed=True)
    rows = [
        ('face', '单面：允许分离与切向滑动，禁止向支撑内部运动。', Assembly((base, part)), 'mesh', ConstraintConfig()),
        ('channel', '对向面：上下均受约束，保留 XY 平面内滑动。', Assembly((base, part, top)), 'mesh', ConstraintConfig()),
        ('corner_twist', '角落六维候选：所有接触单元顶点参与旋转约束。紫色箭头表示角速度轴。',
         Assembly((base, left, back, part)), 'mesh', ConstraintConfig(mode='twist')),
        ('enclosed', '六面局部平移约束：没有非零平移方向。该结论只针对当前刚体接触模型。',
         Assembly((base, top, left, back, replace(left, part_id='right', assembled_tf=pose((.06, 0, 0))),
                   replace(back, part_id='front', assembled_tf=pose((0, .06, 0))), part)), 'mesh', ConstraintConfig()),
        ('near_gap', '零件下方有 0.2 mm 正间隙：没有当前 active 约束，向下候选报告预计间隙闭合参数。',
         Assembly((base, replace(part, assembled_tf=pose((0, 0, .0002))))), 'mesh', ConstraintConfig())]
    shaft = Part('part', cylinder(.0098, .01, 32))
    tube = Part('tube', cylinder(.015, .02, 32, inner_radius=.01), fixed=True)
    rows.append(('shaft_sdf', 'SDF 正间隙轴孔：保留径向 near 法线场和 coaxial mating；不生成虚假的承载/运动约束。',
                 Assembly((shaft, tube), mating_relations=(MatingRelation('part', 'tube', 'coaxial'),)),
                 SDFContactBackend(sdf_config=SDFConfig(max_query_points=50000, validate_mesh_overlap=False)),
                 ConstraintConfig()))
    return rows
