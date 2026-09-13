"""Count qualified WRS grasps against the *current* assembled obstacle set.

Generation is delegated to WRS. Qualification uses actual finger patches and
original meshes. It does not claim IK, approach or carrying-path feasibility.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from time import perf_counter
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import ArrayLike
from scipy.optimize import linprog

from .adapters.wrs_scene import part_from_scene_object, rigid_tf_from_wrs
from .execution import (
    _finger_force_contacts,
    _grasp_contacts,
    generate_execution_grasps,
)
from .geometry.proximity import MeshProximity
from .model import Assembly, AssemblyState, digest, freeze, readonly
from .stability import _generators

if TYPE_CHECKING:
    from wrs.grasp.grasp import Grasp
    from wrs.robots.base.mech_base import MechBase
    from wrs.robots.end_effectors.ee_mixins import JawView


def check_force_closure(
    points: ArrayLike, normals: ArrayLike, *, friction: float = 0.5, cone_sides: int = 16
) -> dict[str, Any]:
    """6D positive-span LP for unilateral hard contacts at supplied pad sites.

    Finite pad corners supply torsional resistance without inventing a free
    moment variable. This is force closure, not a finite load-capacity test.
    """
    p = readonly(points, shape=(None, 3))
    n = readonly(normals, shape=p.shape)
    if not np.isfinite(friction) or friction < 0 or type(cone_sides) is not int or cone_sides < 4:
        raise ValueError("Invalid friction/cone_sides")
    if not len(p) or np.any(np.linalg.norm(n, axis=1) == 0):
        raise ValueError("Nonempty contacts and nonzero normals required")
    n = n / np.linalg.norm(n, axis=1)[:, None]
    rays = np.concatenate([_generators(v, friction, cone_sides) for v in n])
    center = p.mean(0)
    length = max(float(np.linalg.norm(p - center, axis=1).max()), 1e-9)
    lever = np.repeat((p - center) / length, cone_sides if friction else 1, axis=0)
    W = np.vstack((rays.T, np.cross(lever, rays).T))
    rank = int(np.linalg.matrix_rank(W, tol=1e-10))
    if rank < 6:
        return dict(status="not_force_closed", rank=rank, margin=0.0)
    m = len(rays)
    result = linprog(
        np.r_[np.zeros(m), -1.0],
        A_eq=np.vstack((np.column_stack((W, np.zeros(6))), np.r_[np.ones(m), 0.0])),
        b_eq=np.r_[np.zeros(6), 1.0],
        A_ub=np.column_stack((-np.eye(m), np.ones(m))),
        b_ub=np.zeros(m),
        bounds=(0, None),
        method="highs",
        options={"primal_feasibility_tolerance": 1e-9, "dual_feasibility_tolerance": 1e-9},
    )
    if result.status == 2:
        return dict(status="not_force_closed", rank=rank, margin=0.0)
    if not result.success:
        return dict(status="unknown", rank=rank, reason=result.message)
    weights, margin = result.x[:-1], float(result.x[-1])
    residual = max(
        float(np.max(np.abs(W @ weights))),
        abs(float(weights.sum()) - 1),
        float(np.max(margin - weights)),
    )
    if residual > 1e-8:
        return dict(status="unknown", rank=rank, residual=residual)
    return dict(
        status="force_closed" if margin > 1e-8 else "not_force_closed",
        rank=rank,
        margin=margin,
        residual=residual,
        torque_length_m=length,
    )


@dataclass(frozen=True)
class GraspabilityConfig:
    max_grasps: int = 12
    seed: int = 7
    friction: float = 0.5
    cone_sides: int = 16
    contact_roundoff_m: float = 2e-6
    collision_budget: int = 150000

    def __post_init__(self) -> None:
        for k in ("max_grasps", "collision_budget", "cone_sides"):
            if type(getattr(self, k)) is not int or getattr(self, k) < 1:
                raise ValueError(k)
        if self.cone_sides < 4 or type(self.seed) is not int:
            raise ValueError("Invalid cone/seed")
        if not np.isfinite(self.friction) or self.friction < 0:
            raise ValueError("Invalid friction")
        if not np.isfinite(self.contact_roundoff_m) or self.contact_roundoff_m <= 0:
            raise ValueError("Invalid roundoff")


@dataclass(frozen=True)
class GraspabilityResult:
    status: str
    count: int | None
    accepted_ids: tuple[int, ...]
    catalogue_size: int
    input_digest: str
    diagnostics: dict

    def __post_init__(self) -> None:
        object.__setattr__(self, "accepted_ids", tuple(self.accepted_ids))
        object.__setattr__(self, "diagnostics", freeze(self.diagnostics))


class GraspabilityAnalyzer:
    """Owned gripper clone, immutable catalogue snapshots and state-bound caches.

    Optional ``grasps`` maps part IDs to existing WRS Grasp lists. Curved
    contacts without a verified pad patch remain unresolved; antipodal points
    alone are not a certificate of 6D force closure.
    """

    def __init__(
        self,
        assembly: Assembly,
        gripper: MechBase | JawView,
        *,
        config: GraspabilityConfig | None = None,
        grasps: Mapping[str, Iterable[Grasp]] | None = None,
    ) -> None:
        self.assembly = assembly
        self.config = config or GraspabilityConfig()
        self._gripper = gripper.clone()
        self._parts = {p.part_id: p for p in assembly.parts}
        self._provided = {
            k: tuple(freeze(g.to_dict()) for g in v) for k, v in (grasps or {}).items()
        }
        if not set(self._provided).issubset(self._parts):
            raise ValueError("Unknown grasp part")
        self._catalogues = {}
        self._environment = {}
        self._results = {}
        self._backend = MeshProximity(numerical_tol_m=self.config.contact_roundoff_m)
        self.binding = digest(
            (
                assembly,
                self.config,
                self._provided,
                tuple(
                    (part_from_scene_object(l, "link").geometry, rigid_tf_from_wrs(l.tf))
                    for l in self._gripper.runtime_lnks
                ),
                np.asarray(self._gripper.open_dir),
                np.asarray(self._gripper.jaw_range),
            )
        )

    def _catalogue(self, pid):
        if pid in self._catalogues:
            return self._catalogues[pid]
        from wrs.grasp.grasp import Grasp

        cfg = self.config
        part = self._parts[pid]
        records = []
        seen = set()
        supplied = (
            [Grasp.from_dict(dict(g)) for g in self._provided[pid]]
            if pid in self._provided
            else None
        )
        candidates = generate_execution_grasps(
            part, self._gripper, max_grasps=cfg.max_grasps, seed=cfg.seed, candidates=supplied
        )
        for grasp in candidates:
            key = digest((grasp.pose, grasp.qpos))
            if key in seen:
                continue
            seen.add(key)
            probe = self._gripper.clone()
            probe.grip_at(grasp.pose[:3, 3], grasp.pose[:3, :3], grasp.provenance["jaw_width"])
            links = tuple(
                part_from_scene_object(link, f"gripper_{i}")
                for i, link in enumerate(probe.runtime_lnks)
            )
            centers = _grasp_contacts(part, grasp, probe, self._backend, cfg.contact_roundoff_m * 2)
            contacts = _finger_force_contacts(
                part, grasp, probe, self._backend, cfg.contact_roundoff_m
            )
            closure = (
                dict(status="unknown", reason="pad_patch_unresolved")
                if contacts is None
                else check_force_closure(
                    contacts[0], contacts[1], friction=cfg.friction, cone_sides=cfg.cone_sides
                )
            )
            status = closure["status"]
            if status == "force_closed":
                for i, link in enumerate(links):
                    fv = (
                        link.geometry.vertices @ link.assembled_tf[:3, :3].T
                        + link.assembled_tf[:3, 3]
                    )
                    if (
                        i
                        and centers is not None
                        and any(
                            np.min((part.geometry.vertices - p) @ n) >= -cfg.contact_roundoff_m
                            and np.max((fv - p) @ n) <= cfg.contact_roundoff_m
                            for p, n in zip(*centers)
                        )
                    ):
                        continue
                    overlap = self._backend.classify_overlap(
                        part, np.eye(4), link, link.assembled_tf, budget=cfg.collision_budget
                    )
                    # Unqualified touch outside the nominal pads is rejected.
                    if overlap.status != "separated":
                        status = "unknown" if overlap.status == "unknown" else "self_collision"
                        break
            records.append((freeze(grasp.to_dict()), links, freeze(closure), status))
        self._catalogues[pid] = tuple(records)
        return self._catalogues[pid]

    def accepted_grasps(self, result: GraspabilityResult) -> tuple[Grasp, ...]:
        """Fresh WRS Grasp objects for this analyzer's exact recorded query."""
        from wrs.grasp.grasp import Grasp

        if self._results.get(result.input_digest) is not result:
            raise ValueError("Unbound grasp result")
        pid = result.diagnostics["part_id"]
        return tuple(Grasp.from_dict(dict(self._catalogue(pid)[i][0])) for i in result.accepted_ids)

    def analyze(self, state: AssemblyState, part_id: str) -> GraspabilityResult:
        """The upcoming part must be present at its goal pose in ``state``."""
        if part_id not in state.poses or not set(state.poses).issubset(self._parts):
            raise ValueError("Query contains an absent/unknown part")
        key = digest((self.binding, state, part_id))
        if key in self._results:
            return self._results[key]
        started = perf_counter()
        records = self._catalogue(part_id)
        accepted = []
        details = []
        cfg = self.config
        target_tf = state.poses[part_id]
        unresolved = False
        hits = 0
        for gid, (_, links, closure, status) in enumerate(records):
            if status == "force_closed":
                for oid, otf in state.poses.items():
                    if oid == part_id:
                        continue
                    cache_key = digest((part_id, gid, target_tf, oid, otf))
                    if cache_key in self._environment:
                        collision = self._environment[cache_key]
                        hits += 1
                    else:
                        collision = "separated"
                        obstacle = self._parts[oid]
                        ov = obstacle.geometry.vertices @ otf[:3, :3].T + otf[:3, 3]
                        for link in links:
                            tf = target_tf @ link.assembled_tf
                            v = link.geometry.vertices @ tf[:3, :3].T + tf[:3, 3]
                            if np.any(v.min(0) > ov.max(0) + cfg.contact_roundoff_m) or np.any(
                                ov.min(0) > v.max(0) + cfg.contact_roundoff_m
                            ):
                                continue
                            result = self._backend.classify_overlap(
                                link, tf, obstacle, otf, budget=cfg.collision_budget
                            )
                            if result.status != "separated":
                                collision = (
                                    "unknown"
                                    if result.status == "unknown"
                                    else "environment_collision"
                                )
                                break
                        self._environment[cache_key] = collision
                    if collision != "separated":
                        status = collision
                        break
            if status == "force_closed":
                accepted.append(gid)
            if status == "unknown":
                unresolved = True
            details.append(dict(grasp_id=gid, status=status, closure=closure))
        result = GraspabilityResult(
            "partial" if unresolved else "complete",
            None if unresolved else len(accepted),
            tuple(accepted),
            len(records),
            key,
            dict(
                part_id=part_id,
                candidates=details,
                elapsed_s=perf_counter() - started,
                environment_cache_hits=hits,
                known_count=len(accepted),
                scope="fixed_calibrated_catalogue; force_closure_and_goal_gripper_collision_only",
                generation_max_grasps=cfg.max_grasps,
                robot_and_path_pending=True,
            ),
        )
        self._results[key] = result
        return result
