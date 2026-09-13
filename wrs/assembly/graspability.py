"""Compatibility import for wrs.assembly.robotics.graspability; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .robotics import graspability as _implementation

if TYPE_CHECKING:
    from .robotics.graspability import (
        check_force_closure as check_force_closure,
        GraspabilityConfig as GraspabilityConfig,
        GraspabilityResult as GraspabilityResult,
        GraspabilityAnalyzer as GraspabilityAnalyzer,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
