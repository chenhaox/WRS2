"""Paper examples loaded from native WRS manifests; no legacy conversion at runtime."""

from functools import lru_cache
from pathlib import Path

from wrs.assembly import Assembly, AssemblyState, load_assembly

ASSETS = Path(__file__).parent.parent / "assets" / "paper2021"
# Figure keys remain separate even when two experiments reuse one geometry.
# Orders below are inspection prefixes, not newly validated assembly sequences.
CATALOG = {
    "fig08_soma3": ("Fig.8 / Soma 3", "datainfo2", ("Z", "bigL", "smallL 2")),
    "fig09_domino3": ("Fig.9 / Domino 3", "domino_5", ("Domino.001", "Domino.002", "Domino.003")),
    "fig10_burr6": ("Fig.10 / Burr 6", "burrpuzzle", ("u", "3u", "2L right", "2L left", "2u", "1")),
    "fig11_bridge6": ("Fig.11 / Leonardo bridge 6", "bridge", ()),
    "fig12a": ("Fig.12(a) / two blocks — reconstructed", "missing_a", ()),
    "fig12b": ("Fig.12(b) / two blocks — reconstructed", "missing_b", ()),
    "fig12c": ("Fig.12(c) / Soma 3", "datainfo7", ()),
    "fig12d": ("Fig.12(d) / unstable Soma 3", "datainfo0209_3", ()),
    "fig12e": ("Fig.12(e) / Soma 4", "datainfo4", ()),
    "fig12f": ("Fig.12(f) / Domino 3", "domino_5", ()),
    "fig12g": ("Fig.12(g) / offset stack", "domino_7", ()),
    "fig12h": ("Fig.12(h) / supported beam", "domino_8", ()),
    "fig13a": ("Fig.13(a) / BL voxel representation", "single", ()),
    "fig13b": ("Fig.13(b) / BL + Z legacy candidate", "datainfo_ss", ("Z", "bigL")),
    "fig13c_replacement": ("Fig.13(c) / seeded replacement, 5 blocks", "random5", ()),
    "fig13d_replacement": ("Fig.13(d) / seeded replacement, 7 blocks", "random7", ()),
    "fig15a_soma4": ("Fig.15(a) / four-Soma legacy candidate", "datainfo4", ()),
    "fig15b_domino4": ("Fig.15(b) / four-Domino legacy candidate", "domino_6", ()),
    "fig15c_burr6": (
        "Fig.15(c) / Burr 6",
        "burrpuzzle",
        ("u", "3u", "2L right", "2L left", "2u", "1"),
    ),
    "fig15d_bridge6": ("Fig.15(d) / reconstructed bridge", "bridge", ()),
}


@lru_cache(maxsize=40)
def make_case(key: str, *, nominal: bool = True) -> Assembly:
    """Load one immutable scene; raw and nominal are separate source assets."""
    _, name, _ = CATALOG[key]
    if not nominal:
        if name == "bridge":
            raise FileNotFoundError("原始 alframe.stl 缺失；raw 模式不能重建")
        if name.startswith("missing"):
            raise FileNotFoundError("原始 datainfo0209 / datainfo0209_2 场景文件缺失")
        if name.startswith("random"):
            raise FileNotFoundError("论文随机种子和逐次场景未提供；仅提供固定种子的替代数据")
    mode = "nominal" if nominal else "raw"
    return load_assembly(ASSETS / "assemblies" / f"{key}.{mode}.assembly.json")


def prefix_state(assembly: Assembly, stage: int = 0) -> AssemblyState:
    """Select the complete assembly (0) or an explicit installation prefix."""
    order = assembly.provenance["order"]
    if not 0 <= stage <= len(order):
        raise ValueError("stage must be 0 (full) or 1..part_count")
    present = set(order[:stage] if stage else order) | {"ground"}
    return AssemblyState(
        {p.part_id: p.assembled_tf for p in assembly.parts if p.part_id in present}
    )
