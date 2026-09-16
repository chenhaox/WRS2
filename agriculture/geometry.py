"""Small NumPy-only blade/batch helpers; no renderer or physics dependencies."""
import numpy as np

from .spec import LeafShape


def leaf_mesh(length: float, width: float, shape: LeafShape):
    """Pointed lanceolate blade: (vertices, triangle indices), in blade axes.

    +X follows the midrib, +Y crosses the blade, +Z is the upper face. A pair
    of pointed ends avoids degenerate triangles. Two sloping strips meet at
    the folded midrib; curl arches the length, droop lowers the distal end.
    Reverse-wound faces have separate, slightly offset vertices so WRS's
    vertex weld does not cancel the upper/lower surface normals.
    """
    shape.validate()
    if not np.isfinite([length, width]).all() or length <= 0 or width <= 0:
        raise ValueError("leaf length and width must be finite and positive")
    vertices, rows = [], []
    for i, t in enumerate(np.linspace(0, 1, shape.stations)):
        half_width = width * 0.5 * np.sin(np.pi * t) ** shape.profile_power
        z = length * (shape.curl * np.sin(np.pi * t) - shape.droop * t * t)
        ys = [0.0] if i in (0, shape.stations - 1) else [-half_width, 0.0, half_width]
        rows.append(list(range(len(vertices), len(vertices) + len(ys))))
        vertices.extend((length * t, y, z - abs(y) * np.tan(shape.fold)) for y in ys)
    faces = []
    for a, b in zip(rows[:-1], rows[1:]):
        if len(a) == 1:
            faces.extend([(a[0], b[0], b[1]), (a[0], b[1], b[2])])
        elif len(b) == 1:
            faces.extend([(a[0], b[0], a[1]), (a[1], b[0], a[2])])
        else:
            for j in range(2):
                faces.extend([(a[j], b[j], b[j + 1]), (a[j], b[j + 1], a[j + 1])])
    upper = np.asarray(vertices, dtype=float)
    lower = upper.copy()
    upper[:, 2] += shape.thickness * 0.5
    lower[:, 2] -= shape.thickness * 0.5
    fs = np.asarray(faces, dtype=np.uint32)
    return np.concatenate([upper, lower]), np.concatenate([fs, fs[:, ::-1] + len(upper)])


def leaf_batches(spec):
    """Return {color_class: (tree-local vertices, faces)} in shade order."""
    groups = {}
    for leaf in spec.leaves:
        vs, fs = leaf_mesh(leaf.length, leaf.width, spec.leaf_shape)
        vs = vs @ np.asarray(leaf.rotmat).T + leaf.position
        vertices, faces, count = groups.setdefault(leaf.color_class, ([], [], [0]))
        vertices.append(vs)
        faces.append(fs + count[0])
        count[0] += len(vs)
    return {shade: (np.concatenate(groups[shade][0]).astype(np.float32),
                    np.concatenate(groups[shade][1]).astype(np.uint32))
            for shade in sorted(groups)}
