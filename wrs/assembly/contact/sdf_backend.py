"""Batched SDF near-band integration with explicit sampling and sign uncertainty."""
from collections import OrderedDict, defaultdict, deque
from dataclasses import dataclass
from time import perf_counter
import numpy as np
from ..model import ContactAnalysis, ContactPatch, Region, GeometryConfig, digest, to_dict
from ..geometry.proximity import MeshProximity
from ..geometry.preprocess import prepare_mesh
from ..geometry.sdf import SignedDistanceField, Open3DMeshSDF
from ..geometry.mesh_bvh import closest_on_triangle
from ..geometry.planar import cell_regions


@dataclass(frozen=True)
class SDFConfig:
    """SDF-specific controls; shared tolerances remain in ContactConfig.

    ``max_query_points`` is a per-pair integration budget, split between sides.
    Geometry preparation and independent mesh validation have separate costs.
    Open surfaces fail by default; unsigned mode must be requested explicitly.
    """
    nsamples: int = 3
    batch_size: int = 512
    max_query_points: int = 100000
    cache_size: int = 16
    open_surface: str = 'error'
    validate_mesh_overlap: bool = True

    def __post_init__(self):
        for key in ('nsamples', 'batch_size', 'max_query_points', 'cache_size'):
            if not isinstance(getattr(self, key), int) or getattr(self, key) < 1:
                raise ValueError(f'{key} must be a positive integer')
        if self.nsamples % 2 == 0 or self.open_surface not in ('error', 'unsigned'):
            raise ValueError('Use odd nsamples and open_surface=error or unsigned')


def _areas(triangles):
    return np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                   triangles[:, 2]-triangles[:, 0]), axis=1)/2


def _bisect_longest(triangle):
    """Refine the longest edge without repeatedly splitting already short edges."""
    edge = int(np.argmax(np.linalg.norm(np.roll(triangle, -1, axis=0)-triangle, axis=1)))
    a, b, c = triangle[edge], triangle[(edge+1) % 3], triangle[(edge+2) % 3]
    middle = (a+b)/2
    return np.array([a, middle, c]), np.array([middle, b, c])


def _clip_barycentric(scores):
    """Clip a triangle by vertex-linear scalar inequalities, in barycentric space."""
    polygon = np.eye(3)
    for score in scores:
        out = []
        values = polygon @ score
        for i, b in enumerate(polygon):
            a, da, db = polygon[i-1], values[i-1], values[i]
            if (da >= 0) != (db >= 0):
                out.append(a+(b-a)*da/(da-db))
            if db >= 0:
                out.append(b)
        polygon = np.asarray(out).reshape(-1, 3)
        if len(polygon) < 3:
            return np.empty((0, 3))
    return polygon


def _polygon_area_center(poly):
    triangles = np.array([[poly[0], poly[i], poly[i+1]] for i in range(1, len(poly)-1)])
    areas = _areas(triangles)
    total = float(areas.sum())
    return total, (triangles.mean(1)*areas[:, None]).sum(0)/max(total, np.finfo(float).tiny)


class SDFContactBackend:
    """Integrate the near band using a mesh-derived or supplied signed distance field.

    Parameters
    ----------
    sdf_config : SDFConfig, optional
        Query budgets, sign policy and optional independent mesh validation.
    geometry_config : GeometryConfig, optional
        Shared surface preprocessing tolerances.

    Notes
    -----
    This backend reports estimated near/interference bands, never finite active
    contact area from thresholded SDF samples. Normals and grid projections are
    sampled estimates. Mesh validation is separately named in diagnostics and
    is not substituted for a native field's geometry. Instances cache only local
    geometry/fields and are intended for sequential reuse.
    """
    name = 'sdf'

    def __init__(self, *, sdf_config=None, geometry_config=None):
        self.sdf_config = sdf_config or SDFConfig()
        self.geometry_config = geometry_config or GeometryConfig()
        self._mesh = MeshProximity(geometry_config=self.geometry_config)
        self._fields = OrderedDict()
        self.cache_hits = 0

    @property
    def cache_key(self):
        from importlib.metadata import version, PackageNotFoundError
        try:
            runtime = version('open3d')
        except PackageNotFoundError:
            runtime = None
        return digest(('sdf_contact/2', self.sdf_config, self.geometry_config, runtime))

    def prepare(self, model):
        """Return cached surface preprocessing and a field in the same local frame.

        A supplied field is checked at mesh vertices and triangle centroids.
        This detects unit/frame mismatches but does not certify the whole zero set.
        """
        key = model.geometry_key
        if key in self._fields:
            self._fields.move_to_end(key)
            self.cache_hits += 1
            return self._fields[key]
        # SDF integration needs topology/normals, not mesh plane segmentation.
        prep = prepare_mesh(model.geometry, config=self.geometry_config)
        native = model.representations.get('sdf')
        if native is None:
            native = Open3DMeshSDF(prep, nsamples=self.sdf_config.nsamples,
                                    open_surface=self.sdf_config.open_surface)
        else:
            if not isinstance(native, SignedDistanceField):
                raise TypeError('The sdf representation must implement SignedDistanceField')
            points = np.concatenate((prep.mesh.vertices, prep.mesh.vertices[prep.mesh.faces].mean(1)))
            for start in range(0, len(points), self.sdf_config.batch_size):
                check = native.query(points[start:start+self.sdf_config.batch_size])
                tolerance = prep.mesh.geometry_error_m+check.error_m+1e-10
                # A zero gradient at an exact surface corner is acceptable as a
                # scalar registration sample, but not as a contact normal later.
                if not np.all(check.signed) or np.any(np.abs(check.values_m) > tolerance):
                    raise ValueError('Native SDF does not match the supplied surface mesh/frame '
                                     'within declared error, or the grid does not cover it')
        self._fields[key] = (prep, native)
        while len(self._fields) > self.sdf_config.cache_size:
            self._fields.popitem(last=False)
        return prep, native

    def analyze_pair(self, a, tf_a, b, tf_b, *, config):
        """Return bilateral SDF bands and separate whole-solid diagnostics."""
        start = perf_counter()
        pa, fa = self.prepare(a)
        pb, fb = self.prepare(b)
        prepared_at = perf_counter()
        reference, distance = None, None
        native = 'sdf' in a.representations or 'sdf' in b.representations
        if self.sdf_config.validate_mesh_overlap:
            reference = self._mesh.classify_overlap(a.as_part(), tf_a, b.as_part(), tf_b,
                                                    budget=config.max_triangle_tests)
            if not native:
                distance = self._mesh.pair_distance(a.as_part(), tf_a, b.as_part(), tf_b,
                                                    budget=config.max_triangle_tests)
        overlap = (to_dict(reference) if reference is not None and not native else
                   {'status': 'unknown', 'reason': 'native_sdf_requires_global_validation' if native
                    else 'global_mesh_validation_disabled'})
        if (not native and any(not f.metadata.get('signed', False) for f in (fa, fb))
                and overlap.get('reason') != 'disjoint_aabb'):
            overlap = {'status': 'unknown', 'reason': 'unsigned_field_has_no_valid_solid_interior'}
        validated_at = perf_counter()
        patches, reports, negative = [], [], []
        remaining = self.sdf_config.max_query_points
        for reverse in (False, True):
            src, dst, ps, pt, sf, target, ts, tt = (
                (b, a, pb, pa, fb, fa, tf_b, tf_a) if reverse else
                (a, b, pa, pb, fa, fb, tf_a, tf_b))
            limit = remaining if reverse else remaining//2
            found, report, witness = self._integrate(
                a, b, src, dst, ps, pt, sf, target, ts, tt, reverse, config, limit,
                0 if distance is None else distance.lower_bound_m)
            patches.extend(found)
            reports.append(report)
            remaining -= report['sdf_query_points']
            if witness is not None:
                negative.append(witness)
        if negative and overlap['status'] != 'penetrating':
            # Contradictory mesh/SDF evidence stays explicit rather than allowing
            # an approximate signed query to override a whole-mesh certificate.
            overlap = {'status': 'unknown', 'reason': 'negative_sdf_samples_require_validation'}
        diag = {'part_a': a.name, 'part_b': b.name, 'contact_backend': self.name,
                'sdf_a': fa.metadata, 'sdf_b': fb.metadata,
                'overlap': overlap, 'mesh_overlap_reference': to_dict(reference),
                'surface_distance': to_dict(distance), 'negative_sdf_witnesses': negative,
                'curved_coverage': reports, 'planar_coverage': [],
                'unresolved_area_m2': sum(r['unresolved_area_m2'] for r in reports),
                'contact_mode': 'sdf_near_band_estimate',
                'unsupported_contact_features': ['active_area', 'point_line_contact'],
                'area_reference': 'supplied_source_mesh',
                'field_surface_registration': 'sampled_check_for_native_fields'}
        return ContactAnalysis(tuple(patches), (diag,), digest((a.geometry_key, b.geometry_key,
                               tf_a, tf_b, config, self.cache_key)),
                               statistics={'sdf_query_points': sum(r['sdf_query_points'] for r in reports),
                                           'field_cache_hits': self.cache_hits,
                                           'mesh_overlap_triangle_tests': 0 if reference is None else reference.triangle_tests,
                                           'mesh_distance_triangle_tests': 0 if distance is None else distance.triangle_tests,
                                           'timing_s': {'prepare': prepared_at-start,
                                                        'mesh_validation': validated_at-prepared_at,
                                                        'band_integration': perf_counter()-validated_at,
                                                        'backend_total': perf_counter()-start}})

    def _integrate(self, a, b, src, dst, ps, pt, source_field, target, ts, tt,
                   reverse, cfg, limit, global_lower):
        """Breadth-first integration; budget exhaustion retains all pending area."""
        local = ps.mesh.vertices[ps.mesh.faces]
        world = local @ ts[:3, :3].T+ts[:3, 3]
        normals = ps.normals @ ts[:3, :3].T
        error = ps.mesh.geometry_error_m+pt.mesh.geometry_error_m+cfg.pose_error_m
        queue = deque((tri, normals[i], i) for i, tri in enumerate(world))
        entries = defaultdict(list)
        visited = queries = 0
        excluded = uncertain = normal_uncertain = unprocessed = invalid = accepted = 0.0
        reason = 'resolution_reached'
        negative = None
        mesh_target = isinstance(target, Open3DMeshSDF)
        mesh_source = isinstance(source_field, Open3DMeshSDF)
        target_triangles = target.mesh.vertices[target.mesh.faces] if mesh_target else None
        if mesh_target:
            target_world = target.mesh.vertices @ tt[:3, :3].T+tt[:3, 3]
            lo, hi = target_world.min(0), target_world.max(0)
            bounds = np.linalg.norm(np.maximum(np.maximum(world.min(1)-hi, lo-world.max(1)), 0), axis=1)
            keep = bounds <= cfg.near_tol_m+error+cfg.numerical_tol_m
            excluded = float(_areas(world[~keep]).sum())
            queue = deque((world[i], normals[i], i) for i in np.flatnonzero(keep))
        # A whole source-cell distance upper bound can use the convexity of an
        # actual nearest triangle; sign still comes from the SDF of the solid.
        clipped_cells = 0
        def append_cell(poly, center, area, ns, phi, e, lower, upper, signed, q, nt, face_id, target_id):
            nonlocal accepted, normal_uncertain
            if signed and lower > cfg.numerical_tol_m:
                lo, hi = (lower, upper) if phi >= 0 else (-upper, -lower)
            else:
                lo, hi = (-upper, upper) if signed else (lower, upper)
            label = ('near' if signed and lo > cfg.numerical_tol_m else
                     'interference' if signed and hi < -cfg.penetration_tol_m else 'unknown')
            entries[label].append((poly, center, q, ns, nt, area, lo, hi,
                                   face_id, target_id, phi, e, signed))
            accepted += area
            normal_uncertain += area

        while queue:
            fields_per_point = 1 if mesh_source else 2
            # Four samples (centre + vertices), and reserve one true witness
            # query for each possibly clipped polygon in this batch.
            per_cell = 5*fields_per_point
            count = min(len(queue), self.sdf_config.batch_size, (limit-queries)//per_cell, cfg.max_cells-visited)
            if count <= 0:
                unprocessed = float(sum(_areas(np.asarray([item[0]]))[0] for item in queue))
                reason = 'sdf_query_budget_exhausted' if limit-queries < per_cell else 'cell_budget_exhausted'
                break
            batch = [queue.popleft() for _ in range(count)]
            triangles = np.asarray([item[0] for item in batch])
            if mesh_target:
                bounds = np.linalg.norm(np.maximum(np.maximum(triangles.min(1)-hi, lo-triangles.max(1)), 0), axis=1)
                keep = bounds <= cfg.near_tol_m+error+cfg.numerical_tol_m
                excluded += float(_areas(triangles[~keep]).sum())
                visited += int(np.count_nonzero(~keep))
                batch = [item for item, valid in zip(batch, keep) if valid]
                triangles = triangles[keep]
                count = len(batch)
                if not count:
                    continue
            centers = triangles.mean(1)
            radii = np.linalg.norm(triangles-centers[:, None, :], axis=2).max(1)
            probes = np.concatenate((centers[:, None, :], triangles), axis=1).reshape(-1, 3)
            query_local = (probes-tt[:3, 3]) @ tt[:3, :3]
            values = target.query(query_local)
            # Native source normals belong to its zero set, not necessarily to
            # its integration tessellation. Vertex values check the domain;
            # use the cell-interior normal because sharp zero-set corners do
            # not have a unique normal. Charge those queries to this side.
            source_values = None
            if not mesh_source:
                source_values = source_field.query((probes-ts[:3, 3]) @ ts[:3, :3])
            queries += 4*count*fields_per_point
            visited += count
            areas = _areas(triangles)
            pending = []
            for j, (tri, ns, face_id) in enumerate(batch):
                k = 4*j
                sample_slice = slice(k, k+4)
                area, phi, radius = float(areas[j]), float(values.values_m[k]), float(radii[j])
                e = float(values.error_m[sample_slice].max())+error+cfg.numerical_tol_m
                valid = np.all(values.valid[sample_slice]) and (
                    source_values is None or (source_values.valid[k] and np.all(source_values.signed[sample_slice])))
                if not valid:
                    invalid += area
                    continue
                source_normals = np.tile(ns, (4, 1))
                if source_values is not None:
                    ns = source_values.normals_local[k] @ ts[:3, :3].T
                    source_normals = np.tile(ns, (4, 1))
                    e += float(source_values.error_m[sample_slice].max())
                if values.signed[k] and phi < -e-cfg.penetration_tol_m and negative is None:
                    negative = {'sampling_side': 'b' if reverse else 'a',
                                'point_world_m': centers[j].tolist(), 'sdf_m': phi, 'guard_m': e}
                lower = max(0.0, abs(phi)-radius-e, global_lower-e)
                upper = abs(phi)+radius+e
                if lower > cfg.near_tol_m:
                    excluded += area
                    continue
                if mesh_target:
                    target_tri = target_triangles[values.face_ids[k]]
                    tri_local = (tri-tt[:3, 3]) @ tt[:3, :3]
                    witness_upper = max(np.linalg.norm(p-closest_on_triangle(p, target_tri)) for p in tri_local)
                    upper = min(upper, witness_upper+e)
                target_normals = values.normals_local[sample_slice] @ tt[:3, :3].T
                margins = -np.einsum('ij,ij->i', source_normals, target_normals)-np.cos(cfg.normal_angle_rad)
                facing = np.all(margins >= 0)
                band_inside = upper <= cfg.near_tol_m
                sign_unresolved = values.signed[k] and lower <= cfg.numerical_tol_m and abs(phi) > e
                if radius > cfg.surface_resolution_m and (not band_inside or not facing or sign_unresolved):
                    queue.extend((child, ns, face_id) for child in _bisect_longest(tri))
                    continue
                if band_inside and facing:
                    append_cell(tri, centers[j], area, ns, phi, e, lower, upper, bool(values.signed[k]),
                                values.points_local_m[k] @ tt[:3, :3].T+tt[:3, 3],
                                target_normals[0], face_id, int(values.face_ids[k]))
                    continue
                # Cut the terminal cell at the interpolated distance/normal
                # thresholds, instead of painting a whole triangle by its centre.
                vertex_phi = values.values_m[k+1:k+4]
                bary = _clip_barycentric((cfg.near_tol_m-vertex_phi,
                                         cfg.near_tol_m+vertex_phi, margins[1:]))
                if len(bary) < 3:
                    uncertain += area
                    continue
                poly = bary @ tri
                clipped_area, center = _polygon_area_center(poly)
                if clipped_area <= cfg.min_area_m2:
                    uncertain += area
                    continue
                clipped_area = min(area, clipped_area)
                uncertain += area-clipped_area
                clipped_cells += 1
                pending.append((poly, center, clipped_area, ns, e, lower, upper, face_id))
            if pending:
                points = np.asarray([row[1] for row in pending])
                witnesses = target.query((points-tt[:3, 3]) @ tt[:3, :3])
                source_witnesses = None if mesh_source else source_field.query((points-ts[:3, 3]) @ ts[:3, :3])
                queries += len(pending)*fields_per_point
                for j, (poly, center, area, ns, e, lower, upper, face_id) in enumerate(pending):
                    if not witnesses.valid[j] or (source_witnesses is not None and not source_witnesses.valid[j]):
                        invalid += area
                        continue
                    if source_witnesses is not None:
                        ns = source_witnesses.normals_local[j] @ ts[:3, :3].T
                    append_cell(poly, center, area, ns, float(witnesses.values_m[j]), e, lower, upper,
                                bool(witnesses.signed[j]), witnesses.points_local_m[j] @ tt[:3, :3].T+tt[:3, 3],
                                witnesses.normals_local[j] @ tt[:3, :3].T, face_id, int(witnesses.face_ids[j]))
        patches = []
        for label, rows in entries.items():
            groups, boundary_ok = cell_regions([r[0] for r in rows], cfg.numerical_tol_m*4)
            ca, cb, na, nb = (np.asarray([r[i] for r in rows]) for i in (1, 2, 3, 4))
            if reverse:
                ca, cb, na, nb = cb, ca, nb, na
            weights = np.asarray([r[5] for r in rows])
            patches.append(ContactPatch(
                a.name, b.name, 'sdf_surface_a', 'sdf_surface_b', 2, label, 'estimated',
                ca, cb, na, nb,
                tuple(Region(tuple(rows[i][0] for i in ids), loops) for ids, loops in groups),
                (min(r[6] for r in rows), max(r[7] for r in rows)), float(weights.sum()),
                weights=weights, measure_kind='near_band', sampling_side='b' if reverse else 'a',
                provenance={'backend': 'sdf_adaptive/2', 'sdf_provider': target.metadata,
                            'cell_source_prepared_face_ids': [r[8] for r in rows],
                            'cell_target_provider_face_ids': [r[9] for r in rows],
                            'sample_signed_distances_m': [r[10] for r in rows],
                            'sample_error_guard_m': [r[11] for r in rows],
                            'source_face_ids': sorted({i for r in rows for i in ps.source_face_ids[r[8]]}),
                            'boundary_valid': boundary_ok, 'physical_contact_area': False,
                            'normal_test': 'source_cell_interior; target_centre_and_vertices; linear_boundary_clipping',
                            'area_reference': 'source_mesh',
                            'distance_interval': 'signed_sdf' if all(r[12] for r in rows) else 'unsigned'}))
        report = {'sampling_side': 'b' if reverse else 'a', 'source_area_m2': float(ps.areas_m2.sum()),
                  'estimated_band_area_m2': accepted, 'excluded_area_m2': excluded,
                  'bounded_band_area_m2': 0.0, 'unprocessed_area_m2': unprocessed,
                  'invalid_field_area_m2': invalid, 'boundary_uncertain_area_m2': uncertain,
                  'normal_uncertain_area_m2': normal_uncertain,
                  'unresolved_area_m2': uncertain+normal_uncertain+unprocessed+invalid,
                  'visited_cells': visited, 'sdf_query_points': queries, 'reason': reason,
                  'clipped_boundary_cells': clipped_cells,
                  'normal_test': 'sampled', 'quality': 'estimated',
                  'area_accounting': 'accepted plus uncertain plus excluded plus invalid plus unprocessed equals source area; normal uncertainty overlaps accepted area'}
        return patches, report, negative
