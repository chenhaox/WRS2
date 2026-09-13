"""Compatibility import for wrs.assembly.mechanics.stability_sweep; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .mechanics import stability_sweep as _implementation

if TYPE_CHECKING:
    from .mechanics.stability_sweep import (
        StabilitySweepConfig as StabilitySweepConfig,
        DirectionalLimit as DirectionalLimit,
        StabilitySweepResult as StabilitySweepResult,
        disturbance_directions as disturbance_directions,
        DirectionalStabilityAnalyzer as DirectionalStabilityAnalyzer,
        analyze_directional_stability as analyze_directional_stability,
        limit_wrench as limit_wrench,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
