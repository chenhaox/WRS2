"""Compatibility import for wrs.assembly.geometry.primitives; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .geometry import primitives as _implementation

if TYPE_CHECKING:
    from .geometry.primitives import (
        pose as pose,
        box as box,
        rectangle as rectangle,
        rectangular_ring as rectangular_ring,
        cylinder as cylinder,
        sphere as sphere,
        combine as combine,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
