"""Migration of old asp JSON + STL without importing its Panda3D modules."""

from __future__ import annotations

import json
from os import PathLike
from pathlib import Path
from typing import Any

import numpy as np

from wrs.assembly.io import read_mesh, unit_scale
from wrs.assembly.model import Assembly, Part
from wrs.assembly.geometry.transforms import rotation_xyz


def legacy_asset_inventory(
    legacy_root: str | PathLike[str], name: str, *, length_unit: str
) -> dict[str, Any]:
    """Inspect all referenced assets. Missing models remain explicit entries."""
    scale = unit_scale(length_unit)
    root = Path(legacy_root)
    data = json.loads((root / "asp" / "data" / name).read_text(encoding="utf-8"))
    entries = []
    for key, value in data.items():
        model = {"smallL 2": "smallL_2", "Cross": "cross", "Crossleft": "crossleft"}.get(key, key)
        model = model.replace(" ", "_")
        if "Domino" in key:
            model = "domino"
        if "Alframe" in key:
            model = "alframe"
        path = root / "asp" / "objects" / f"{model}.stl"
        if not path.exists():
            path = path.with_name(model[0].lower() + model[1:] + ".stl")
        tf = np.eye(4)
        tf[:3, :3] = rotation_xyz(value["rotation"])
        tf[:3, 3] = np.asarray(value["location"]) * scale
        entry = {
            "part_id": key,
            "path": str(path.resolve()),
            "exists": path.is_file(),
            "assembled_tf_m": tf.tolist(),
            "mass_kg": None,
            "friction": None,
        }
        if path.is_file():
            mesh = read_mesh(path, length_unit=length_unit)
            entry.update(
                vertex_count=len(mesh.vertices),
                triangle_count=len(mesh.faces),
                extents_m=np.ptp(mesh.vertices, axis=0).tolist(),
            )
        entries.append(entry)
    return {
        "name": name,
        "input_length_unit": length_unit,
        "rotation_convention": "extrinsic xyz radians; Rz @ Ry @ Rx",
        "part_count": len(entries),
        "parts": entries,
        "unit_note": "Explicit user assumption; STL cannot verify its own unit",
    }


def load_legacy_assembly(
    legacy_root: str | PathLike[str], name: str, *, length_unit: str
) -> Assembly:
    """Load legacy instances with explicit units; fail on any missing asset."""
    report = legacy_asset_inventory(legacy_root, name, length_unit=length_unit)
    missing = [p["path"] for p in report["parts"] if not p["exists"]]
    if missing:
        raise FileNotFoundError("Missing legacy assets: " + ", ".join(sorted(set(missing))))
    meshes, parts = {}, []
    for p in report["parts"]:
        if p["path"] not in meshes:
            meshes[p["path"]] = read_mesh(p["path"], length_unit=length_unit)
        parts.append(Part(p["part_id"], meshes[p["path"]], p["assembled_tf_m"]))
    return Assembly(tuple(parts), provenance=report)
