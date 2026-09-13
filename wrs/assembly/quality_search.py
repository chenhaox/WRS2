"""Compatibility import for wrs.assembly.planning.quality_search; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .planning import quality_search as _implementation

if TYPE_CHECKING:
    from .planning.quality_search import (
        QualityTransition as QualityTransition,
        QualitySearchConfig as QualitySearchConfig,
        quality_depth_first as quality_depth_first,
        QualitySequenceResult as QualitySequenceResult,
        plan_quality_sequence as plan_quality_sequence,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
