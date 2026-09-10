"""WRS public aliases, loaded on demand so geometry can run headlessly."""
from importlib import import_module

_MODULES = {
    'np': 'numpy', 'key': 'wrs.viewer.key',
    'wum': 'wrs.utils.math', 'wuh': 'wrs.utils.helper', 'wuc': 'wrs.utils.constant',
    'wgg': 'wrs.geom.geometry', 'wgl': 'wrs.geom.loader',
    'wss': 'wrs.scene.scene', 'wsso': 'wrs.scene.scene_object',
    'wssop': 'wrs.scene.scene_object_primitive', 'wsrm': 'wrs.scene.render_model',
    'wsgop': 'wrs.scene.geometry_ops', 'wvw': 'wrs.viewer.world',
    'wcm': 'wrs.collider.mj_collider', 'wccs': 'wrs.collider.cpu_simd',
    'wgab': 'wrs.grasp.antipodal', 'wgpp': 'wrs.grasp.polypodal',
    'wgmc': 'wrs.grasp.monocontact', 'wgpl': 'wrs.grasp.placement',
    'wgr': 'wrs.grasp.reasoner', 'wgs': 'wrs.grasp.serialize',
    'wmppc': 'wrs.motion.core.planning_context', 'wmpr': 'wrs.motion.probabilistic.rrt',
    'wmpp': 'wrs.motion.probabilistic.prm', 'wmic': 'wrs.motion.interpolation.cartesian',
    'wmij': 'wrs.motion.interpolation.joint', 'wmttg': 'wrs.motion.trajectory.totg',
    'wmpad': 'wrs.motion.primitives.approach_depart',
    'wmpp_pickplace': 'wrs.manipulation.pick_place', 'wma': 'wrs.manipulation.arm',
    'khi_rs007l': 'wrs.robots.manipulators.kawasaki.rs007l.rs007l',
    'xarm_lite6': 'wrs.robots.manipulators.xarm.lite6.lite6',
    'or_2fg7': 'wrs.robots.end_effectors.onrobot.or_2fg7.or_2fg7',
    'xyt': 'wrs.robots.vehicle.xytheta',
}
_SYMBOLS = {
    'Grasp': ('wrs.grasp.grasp', 'Grasp'),
    'MotionData': ('wrs.motion.core.motion_data', 'MotionData'),
    'gen_pick_and_place': ('wrs.manipulation.pick_place', 'gen_pick_and_place'),
    'Arm': ('wrs.manipulation.arm', 'Arm'),
    'SingleArmManipulation': ('wrs.manipulation.arm', 'SingleArmManipulation'),
}
# The previous __all__ included an undefined wmttp; actual aliases are preserved.
__all__ = list(_MODULES) + list(_SYMBOLS)


def __getattr__(name):
    if name in _MODULES:
        value = import_module(_MODULES[name])
    elif name in _SYMBOLS:
        module, attr = _SYMBOLS[name]
        value = getattr(import_module(module), attr)
    else:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))
