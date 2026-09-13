"""Optional WRS contact scenes; rendering never makes contact decisions."""

from __future__ import annotations

from typing import TYPE_CHECKING

from ..model import Assembly, AssemblyState, ContactAnalysis

if TYPE_CHECKING:
    from wrs.scene.scene import Scene


COLORS = {"active": "#078b70", "near": "#e89420", "interference": "#d74850", "unknown": "#8466b4"}


def build_wrs_scene(assembly: Assembly, state: AssemblyState, analysis: ContactAnalysis) -> Scene:
    """Build a WRS scene lazily, with original meshes and evidence overlays.

    This optional bridge creates render models only. It does not use collider
    proxies, start the viewer or modify assembly state. Call World.run yourself.
    """
    from wrs import wss, wssop

    scene = wss.Scene()
    for i, part in enumerate(assembly.parts):
        if part.part_id not in state.poses:
            continue
        obj = wssop.mesh(
            part.geometry.vertices,
            part.geometry.faces,
            rgb=(0.35, 0.52, 0.7) if i % 2 else (0.68, 0.73, 0.8),
            alpha=0.24,
            name=part.part_id,
        )
        obj.tf = state.poses[part.part_id]
        scene.add(obj)
    for k, patch in enumerate(analysis.patches):
        color = COLORS[patch.classification]
        rgb = tuple(int(color[i : i + 2], 16) / 255 for i in (1, 3, 5))
        vs, fs = [], []
        for region in patch.regions:
            for cell in region.cells_world_m:
                if patch.dimension == 2:
                    start = len(vs)
                    vs.extend(cell)
                    fs.extend([[start, start + i, start + i + 1] for i in range(1, len(cell) - 1)])
                elif len(cell) == 2:
                    scene.add(wssop.cylinder(cell[0], cell[1], radius=0.00025, rgb=rgb))
        if fs:
            scene.add(wssop.mesh(vs, fs, rgb=rgb, alpha=0.85, name=f"contact_{k}"))
        stride = max(1, len(patch.points_a_world_m) // 12)
        for p, n in zip(patch.points_a_world_m[::stride], patch.normals_a_world[::stride]):
            scene.add(
                wssop.arrow(p, p + n * 0.008, shaft_radius=0.00015, head_radius=0.0004, rgb=rgb)
            )
    return scene
