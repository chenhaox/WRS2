"""Compatibility import for wrs.assembly.motion.part_motion; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .motion import part_motion as _implementation

if TYPE_CHECKING:
    from .motion.part_motion import (
        MotionConfig as MotionConfig,
        RemovalAction as RemovalAction,
        ContactPolicy as ContactPolicy,
        PathValidation as PathValidation,
        RemovalResult as RemovalResult,
        interpolate_pose as interpolate_pose,
        sample_path as sample_path,
        validate_object_path as validate_object_path,
        plan_removal as plan_removal,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
