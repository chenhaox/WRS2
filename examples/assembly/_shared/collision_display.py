"""Convert collision evidence to the same point/line/face contact overlays."""
import numpy as np


def collision_layers(checker, models, result, resolution_m):
    layers = []
    if result["status"] == "penetrating":
        regions = checker.penetration_regions(
            *models, resolution_m=resolution_m, max_query_points=500000)
        for side in regions["sides"]:
            layers.append(dict(name=f"penetration {side['sampling_side']}",
                               kind="interference", dimension=2,
                               cells=side["cells_world_m"], points=[]))
        print("穿透表面区域（不是相交体积）:", regions["timing_s"], flush=True)
    elif result["status"] == "touching":
        touch = checker.touch_regions(*models)
        for dimension in sorted(set(touch["cell_dimensions"])):
            cells = [c for c, d in zip(touch["cells_world_m"], touch["cell_dimensions"])
                     if d == dimension]
            layers.append(dict(name="touch", kind="active", dimension=dimension,
                               cells=cells, points=np.vstack(cells) if cells else []))
        print("触碰区域:", touch["quality"], touch["timing_s"], flush=True)
    return layers
