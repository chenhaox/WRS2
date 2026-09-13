"""Compatibility import for wrs.assembly.robotics.execution; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .robotics import execution as _implementation

if TYPE_CHECKING:
    from .robotics.execution import (
        ExecutionConfig as ExecutionConfig,
        ExecutionArm as ExecutionArm,
        ExecutionResult as ExecutionResult,
        generate_execution_grasps as generate_execution_grasps,
        ExecutionWorkcell as ExecutionWorkcell,
        validate_execution as validate_execution,
        replay_execution as replay_execution,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
