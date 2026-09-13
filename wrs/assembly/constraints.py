"""Compatibility import for wrs.assembly.motion.constraints; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .motion import constraints as _implementation

if TYPE_CHECKING:
    from .motion.constraints import (
        ConstraintConfig as ConstraintConfig,
        LocalConstraints as LocalConstraints,
        rebase_twist as rebase_twist,
        contact_constraints as contact_constraints,
        MotionCandidates as MotionCandidates,
        candidate_motions as candidate_motions,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
