"""Compatibility import for wrs.assembly.motion.directions; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .motion import directions as _implementation

if TYPE_CHECKING:
    from .motion.directions import (
        DirectionConfig as DirectionConfig,
        DirectionResult as DirectionResult,
        fibonacci_directions as fibonacci_directions,
        solve_directions as solve_directions,
        assembly_directions as assembly_directions,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
