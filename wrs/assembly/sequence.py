"""Compatibility import for wrs.assembly.planning.sequence; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .planning import sequence as _implementation

if TYPE_CHECKING:
    from .planning.sequence import (
        HandlingCapability as HandlingCapability,
        AuxiliarySupport as AuxiliarySupport,
        SequenceConfig as SequenceConfig,
        SequenceStep as SequenceStep,
        SequenceResult as SequenceResult,
        SequenceEvaluator as SequenceEvaluator,
        plan_sequence as plan_sequence,
        replay_sequence as replay_sequence,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
