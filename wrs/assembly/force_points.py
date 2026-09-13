"""Compatibility import for wrs.assembly.mechanics.force_points; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .mechanics import force_points as _implementation

if TYPE_CHECKING:
    from .mechanics.force_points import (
        ForcePoints as ForcePoints,
        prepare_force_points as prepare_force_points,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
