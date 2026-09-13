"""Compatibility import for wrs.assembly.mechanics.equilibrium; use that path in new code."""

import sys
from typing import TYPE_CHECKING
from .mechanics import equilibrium as _implementation

if TYPE_CHECKING:
    from .mechanics.equilibrium import (
        ExternalWrench as ExternalWrench,
        LoadCase as LoadCase,
        SupportCandidate as SupportCandidate,
        StabilityConfig as StabilityConfig,
        EquilibriumCase as EquilibriumCase,
        EquilibriumResult as EquilibriumResult,
        check_equilibrium as check_equilibrium,
        SupportSearchResult as SupportSearchResult,
        find_support_requirements as find_support_requirements,
    )

# Share the module object, including caches and monkeypatches; no call wrapper.
sys.modules[__name__] = _implementation
