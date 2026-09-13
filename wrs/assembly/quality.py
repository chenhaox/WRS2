"""Compatibility import for wrs.assembly.planning.quality; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .planning import quality as _implementation

if TYPE_CHECKING:
    from .planning.quality import (
        AssemblabilityScore as AssemblabilityScore,
        score_assemblability as score_assemblability,
        StepQuality as StepQuality,
        sequence_quality as sequence_quality,
        quality_upper_bound as quality_upper_bound,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
