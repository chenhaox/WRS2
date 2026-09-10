"""Plane charts and convex cell clipping; holes are represented by cell unions."""
from collections import defaultdict
import numpy as np
from scipy.spatial import cKDTree


def plane_basis(normal):
    """Return a (3,2) right-handed orthonormal plane basis, u cross v = n."""
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    axis = np.eye(3)[np.argmin(np.abs(n))]
    u = np.cross(axis, n)
    u /= np.linalg.norm(u)
    return np.column_stack((u, np.cross(n, u)))


def clean_polygon(poly, tol):
    """Remove adjacent coincident vertices without filling concavities."""
    out = []
    for p in np.asarray(poly):
        if not out or np.linalg.norm(p - out[-1]) > tol:
            out.append(p)
    if len(out) > 1 and np.linalg.norm(out[0] - out[-1]) <= tol:
        out.pop()
    return np.asarray(out, dtype=float).reshape(-1, 2)


def polygon_measure(poly):
    """Return signed area (m²) and centroid (m), including degenerate cells."""
    p = np.asarray(poly)
    if not len(p):
        return 0.0, np.zeros(2)
    if len(p) < 3:
        return 0.0, p.mean(axis=0)
    # Centering avoids cancellation for small regions far from chart origin.
    origin = p[0]
    q = p - origin
    nxt = np.roll(q, -1, axis=0)
    cross = q[:, 0] * nxt[:, 1] - q[:, 1] * nxt[:, 0]
    area = cross.sum() * 0.5
    if abs(area) <= np.finfo(float).tiny:
        return 0.0, p.mean(axis=0)
    center = ((q + nxt) * cross[:, None]).sum(axis=0) / (6 * area)
    return float(area), center + origin


def clip_halfplane(poly, normal, offset, tol):
    """Clip a convex cell to dot(p, normal)+offset >= 0 (chart metres)."""
    p = np.asarray(poly, dtype=float).reshape(-1, 2)
    if not len(p):
        return p
    vals = p @ normal + offset
    vals[np.abs(vals) <= tol] = 0
    out = []
    for i in range(len(p)):
        a, b = p[i - 1], p[i]
        da, db = vals[i - 1], vals[i]
        if (da >= 0) != (db >= 0):
            out.append(a + (b - a) * da / (da - db))
        if db >= 0:
            out.append(b)
    return clean_polygon(out, tol)


def convex_intersection(a, b, tol):
    """Intersect two convex polygons, retaining line and point intersections."""
    out = clean_polygon(a, tol)
    b = np.asarray(b)
    if polygon_measure(b)[0] < 0:
        b = b[::-1]
    for i in range(len(b)):
        edge = b[(i + 1) % len(b)] - b[i]
        length = np.linalg.norm(edge)
        if length <= tol:
            continue
        n = np.array([-edge[1], edge[0]]) / length
        out = clip_halfplane(out, n, -np.dot(n, b[i]), tol)
        if not len(out):
            break
    return out


def trace_edges(edges):
    """Trace oriented index edges into every closed ring; report bad topology."""
    outgoing = defaultdict(list)
    for a, b in edges:
        outgoing[a].append(b)
    if any(len(v) != 1 for v in outgoing.values()):
        return (), False
    remaining = set(edges)
    loops = []
    while remaining:
        start, cur = min(remaining)
        loop = [start]
        remaining.remove((start, cur))
        while cur != start:
            loop.append(cur)
            if cur not in outgoing or (cur, outgoing[cur][0]) not in remaining:
                return tuple(loops), False
            nxt = outgoing[cur][0]
            remaining.remove((cur, nxt))
            cur = nxt
        loops.append(tuple(loop))
    return tuple(loops), True


def cell_regions(cells, tol):
    """Group oriented 2D/3D cells and recover holes, splitting T junctions.

    Returns (groups, valid), where each group is (cell_indices, boundary_loops).
    The original cells remain authoritative even if a boundary is ambiguous.
    Memory is linear in cell vertices; no all-pairs matrix is constructed.
    """
    if not cells:
        return (), True
    ids, points, indexed = {}, [], []
    origin = np.asarray(cells[0][0])
    for cell in cells:
        if np.shape(cell)[1] == 2 and polygon_measure(cell)[0] < 0:
            cell = cell[::-1]
        seq = []
        for p in cell:
            key = tuple(np.rint((p - origin) / tol).astype(np.int64))
            if key not in ids:
                ids[key] = len(points)
                points.append(p)
            seq.append(ids[key])
        indexed.append(seq)
    points = np.asarray(points)
    tree = cKDTree(points)
    edge_owners = defaultdict(list)
    cell_edges = []
    for cid, seq in enumerate(indexed):
        edges = []
        for i, a in enumerate(seq):
            b = seq[(i + 1) % len(seq)]
            d = points[b] - points[a]
            length2 = np.dot(d, d)
            if length2 <= tol * tol:
                continue
            nearby = np.asarray(sorted(tree.query_ball_point((points[a]+points[b])/2,
                                                             np.sqrt(length2)/2 + 2*tol)))
            ts = (points[nearby] - points[a]) @ d / length2
            dist = np.linalg.norm(points[nearby] - points[a] - ts[:, None] * d, axis=1)
            inside = np.flatnonzero((ts >= -tol / np.sqrt(length2)) &
                                    (ts <= 1 + tol / np.sqrt(length2)) & (dist <= 2 * tol))
            ordered = nearby[inside[np.argsort(ts[inside], kind='stable')]]
            for x, y in zip(ordered[:-1], ordered[1:]):
                if x != y and np.linalg.norm(points[x] - points[y]) > tol * .5:
                    edges.append((int(x), int(y)))
                    edge_owners[tuple(sorted((int(x), int(y))))].append(cid)
        cell_edges.append(edges)
    parent = list(range(len(cells)))

    def find(i):
        while i != parent[i]:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for owners in edge_owners.values():
        for other in owners[1:]:
            parent[find(other)] = find(owners[0])
    components = defaultdict(list)
    for i in range(len(cells)):
        components[find(i)].append(i)
    groups, valid = [], all(len(owners) <= 2 for owners in edge_owners.values())
    for members in components.values():
        boundary = []
        for i in members:
            boundary.extend(e for e in cell_edges[i] if len(edge_owners[tuple(sorted(e))]) == 1)
        loops, ok = trace_edges(boundary)
        valid &= ok
        groups.append((tuple(members), tuple(points[list(loop)] for loop in loops)))
    return tuple(groups), valid
