"""Declared, dataset-specific nominal repair of the six archived burr pieces.

These profiles align matching slot planes and seat the unnotched key. They are
an inferred fit model, not recovered author CAD or a general mesh repair rule.
The raw STL files and raw manifests remain the source of measured geometry.
"""

from itertools import product
from typing import Any

import numpy as np

from wrs.assembly import MeshData
from wrs.assembly.geometry.preprocess import prepare_mesh


# Coordinates are in the STL's assembled frame, in millimetres. Keep the
# existing asymmetric reliefs and key clearances; do not round every feature
# onto a uniform voxel grid. The 2L-right slot at -28.6 mm remains unchanged.
NOMINAL_PLANES_MM = {
    "3u": (
        (-5.5, -0.544936, 0.5, 4.955063),
        (-28.27907943725586, -17.0, -11.5, 0.0, 6.0, 16.72092056274414),
        (-34.0, -28.5, -23.0),
    ),
    "2L_left": (
        (-22.0, -5.5, 0.0, 5.11191, 10.61191, 23.0),
        (-17.0, -11.5, -5.5),
        (-28.5, -23.0, -17.0),
    ),
    "2L_right": (
        (-11.5, -5.5, 0.0),
        (-11.5, -5.5, 0.0),
        (-45.0, -28.6, -23.0, -17.0, -10.0, 0.0),
    ),
    "2u": (
        (-21.9891357421875, -11.5, -5.5, 5.11191, 10.61191, 23.0108642578125),
        (-5.5, 0.5, 6.0),
        (-28.5, -23.0, -17.0),
    ),
    "u": (
        (0.022694, 0.5, 5.11191, 10.61191),
        (-11.5, 0.0),
        (-45.0, -34.0, -10.0, 0.0),
    ),
    # Seat the key's bottom at -23 mm, preserving its height and side gaps.
    "1": ((-5.4, 5.01), (-28.194650, 16.805350), (-23.0, -10.172723770141601)),
}
MAX_VERTEX_CHANGE_M = 0.0003


def _rectilinear_boundary(vertices: np.ndarray, faces: np.ndarray) -> MeshData:
    """Retriangulate the snapped solid on its nonuniform coordinate planes.

    Slot cavities are kept using integer winding at cell centers. This removes
    the degenerate triangles/T junctions created when the warped 3u slot is
    made planar; it does not fill holes or replace the solid by its convex hull.
    The small plane count is specific to this dataset; this runs only offline.
    """
    axes = [np.unique(vertices[:, i]) for i in range(3)]
    cells = np.asarray(list(product(*(range(len(a) - 1) for a in axes))))
    centers = np.column_stack(
        [(axis[cells[:, i]] + axis[cells[:, i] + 1]) / 2 for i, axis in enumerate(axes)]
    )
    relative = vertices[faces][None, :, :, :] - centers[:, None, None, :]
    a, b, c = relative.transpose(2, 0, 1, 3)
    la, lb, lc = np.linalg.norm(relative, axis=3).transpose(2, 0, 1)

    def dot(x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return np.einsum("...i,...i->...", x, y)

    numerator = dot(a, np.cross(b, c))
    denominator = la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb
    winding = np.arctan2(numerator, denominator).sum(axis=1) / (2 * np.pi)
    if not np.allclose(winding, np.rint(winding), atol=1e-8, rtol=0):
        raise ValueError("Burr profile did not produce a well-defined rectilinear solid")
    if np.any(np.abs(np.rint(winding)) > 1):
        raise ValueError("Overlapping shells in the nominal burr profile")
    occupied = set(map(tuple, cells[np.abs(winding) > 0.5]))
    if not occupied:
        raise ValueError("Empty nominal burr solid")
    boundary_vertices, boundary_faces = [], []
    vertex_ids = {}
    for cell in sorted(occupied):
        for normal_axis in range(3):
            for sign in (-1, 1):
                neighbor = list(cell)
                neighbor[normal_axis] += sign
                if tuple(neighbor) in occupied:
                    continue
                u, v = (normal_axis + 1) % 3, (normal_axis + 2) % 3
                quad = []
                for du, dv in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    index = list(cell)
                    index[normal_axis] += int(sign > 0)
                    index[u] += du
                    index[v] += dv
                    key = tuple(index)
                    if key not in vertex_ids:
                        vertex_ids[key] = len(boundary_vertices)
                        boundary_vertices.append([axes[i][key[i]] for i in range(3)])
                    quad.append(vertex_ids[key])
                if sign < 0:
                    quad.reverse()
                boundary_faces.extend(((quad[0], quad[1], quad[2]), (quad[0], quad[2], quad[3])))
    return MeshData(boundary_vertices, boundary_faces)


def repair_burr_mesh(source: MeshData, name: str) -> tuple[MeshData, dict[str, Any]]:
    """Return the declared nominal mesh and an auditable displacement record."""
    planes = NOMINAL_PLANES_MM[name]
    vertices = source.vertices.copy()
    for axis, values in enumerate(planes):
        levels = np.asarray(values) * 0.001
        indices = np.argmin(np.abs(vertices[:, axis, None] - levels), axis=1)
        vertices[:, axis] = levels[indices]
    maximum = float(np.linalg.norm(vertices - source.vertices, axis=1).max())
    if maximum > MAX_VERTEX_CHANGE_M:
        raise ValueError(f"{name}: source differs from the burr profile by {maximum:g} m")
    original = prepare_mesh(source)
    if not original.is_closed or not original.orientation_reliable:
        raise ValueError(f"{name}: source must have a closed, consistently oriented boundary")
    # Keep the original source indexing for the measured per-vertex change;
    # preprocessing may reorder vertices, so do not reuse its face indices.
    repaired = _rectilinear_boundary(vertices, source.faces)
    prepared = prepare_mesh(repaired)
    if not prepared.is_closed or not prepared.orientation_reliable:
        raise ValueError(f"{name}: nominal reconstruction is not a closed oriented mesh")
    record = {
        "method": "burr_slot_plane_alignment/1",
        "source_geometry_id": source.geometry_id,
        "max_world_vertex_change_m": maximum,
        "vertex_change_limit_m": MAX_VERTEX_CHANGE_M,
        "nominal_planes_mm": planes,
        "surface_rebuilt": "conforming boundary of occupied nonuniform grid cells",
        "interpretation": "declared nominal fit; not recovered original CAD",
    }
    return repaired, record
