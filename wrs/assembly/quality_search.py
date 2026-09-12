"""Forward quality-guided DFS, with bounded supports and existing M2 replay."""
from dataclasses import dataclass, field, replace
from itertools import combinations, chain, islice
from time import perf_counter
import numpy as np
from .model import AssemblyState, digest, freeze
from .quality import StepQuality, score_assemblability, sequence_quality, quality_upper_bound
from .directions import DirectionConfig, assembly_directions
from .sequence import SequenceConfig, SequenceEvaluator, SequenceResult, replay_sequence, _balanced
from .stability_sweep import StabilitySweepConfig, DirectionalStabilityAnalyzer


@dataclass(frozen=True)
class QualityTransition:
    context: object
    quality: StepQuality
    payload: object = None


@dataclass(frozen=True)
class QualitySearchConfig:
    support_penalty: float = 100.
    prune: bool = True
    max_expansions: int = 1000
    time_limit_s: float = 180.
    zero_stability_tol: float = 1e-8
    require_final_without_support: bool = True
    sequence: SequenceConfig = field(default_factory=lambda:SequenceConfig(max_auxiliary_resources=1))
    sweep: StabilitySweepConfig = field(default_factory=StabilitySweepConfig)
    direction: DirectionConfig = field(default_factory=DirectionConfig)

    def __post_init__(self):
        if not np.isfinite(self.support_penalty) or self.support_penalty < 1: raise ValueError('Invalid support penalty')
        if type(self.max_expansions) is not int or self.max_expansions < 1: raise ValueError('Invalid expansion budget')
        if not np.isfinite(self.time_limit_s) or self.time_limit_s <= 0: raise ValueError('Invalid time budget')
        if not np.isfinite(self.zero_stability_tol) or self.zero_stability_tol < 0: raise ValueError('Invalid zero tolerance')
        if type(self.prune) is not bool or type(self.require_final_without_support) is not bool: raise ValueError('Expected bool')
        if self.sweep.mode != 'wrench': raise ValueError('Quality uses a dimensionless 6D radial stability score')
        if digest(self.sequence.stability) != digest(self.sweep.stability):
            raise ValueError('Sequence and sweep must use the same stability/contact model')


def quality_depth_first(items, expand, *, initial_context=None, config=None, accept_complete=None):
    """Decoupled DFS kernel; expand(context,item) yields QualityTransition(s).

    Infeasible edges yield nothing. Numerical unknowns belong in the oracle's
    diagnostics. No state-only visited set: minima/support counts depend on
    the prefix. The callback validates every improving complete candidate.
    """
    cfg = config or QualitySearchConfig(); items = tuple(sorted(items))
    if not items or len(set(items)) != len(items): raise ValueError('Distinct nonempty items required')
    started = perf_counter(); best = None; best_score = -1.; expanded = 0; pruned = 0
    complete = True; leaves = 0; trace = []

    def visit(context, remaining, order, qualities, payloads):
        nonlocal best, best_score, expanded, pruned, complete, leaves
        if perf_counter()-started >= cfg.time_limit_s:
            complete = False; return
        if qualities:
            upper = quality_upper_bound(qualities, support_penalty=cfg.support_penalty, remaining_steps=len(remaining))
            if cfg.prune and upper <= best_score:
                pruned += 1
                trace.append(dict(order=order, status='bound_pruned', upper_bound=upper))
                return
        if not remaining:
            leaves += 1; value = sequence_quality(qualities, support_penalty=cfg.support_penalty)
            if value > best_score and (accept_complete is None or accept_complete(payloads)):
                best_score = value; best = (order, qualities, payloads)
                trace.append(dict(order=order, status='incumbent', score=value))
            return
        if expanded >= cfg.max_expansions:
            complete = False; return
        expanded += 1
        for item in remaining:
            if perf_counter()-started >= cfg.time_limit_s: complete = False; return
            for child in expand(context, item):
                if perf_counter()-started >= cfg.time_limit_s: complete = False; return
                q = child.quality
                if q.graspability == 0 or q.assemblability == 0: continue
                prefix = qualities+(q,)
                trace.append(dict(order=order+(item,), status='evaluated', quality=q,
                    prefix_score=sequence_quality(prefix, support_penalty=cfg.support_penalty)))
                visit(child.context, tuple(x for x in remaining if x != item), order+(item,), prefix, payloads+(child.payload,))
                if not complete: return
    visit(initial_context, items, (), (), ())
    return dict(best=best, score=None if best is None else best_score, search_complete=complete,
                expansions=expanded, pruned=pruned, leaves=leaves, trace=trace, elapsed_s=perf_counter()-started)


@dataclass(frozen=True)
class QualitySequenceResult:
    status: str
    plan: SequenceResult | None
    score: float | None
    qualities: tuple
    diagnostics: dict
    input_digest: str
    config: QualitySearchConfig

    def __post_init__(self):
        object.__setattr__(self, 'qualities', tuple(self.qualities))
        object.__setattr__(self, 'diagnostics', freeze(self.diagnostics))


def plan_quality_sequence(assembly, grasp_analyzer, goal_state=None, *, supports=(), config=None):
    """Add parts from the fixed base; maximize Chen's score over tested orders.

    A is the finite ASP_OLD cone-class score. S is the minimum sampled,
    capped, dimensionless radial load factor, with no auxiliary support.
    S=0 requires a separately validated finite support subset. G counts
    qualified grasps in the analyzer's fixed catalogue. Returned ``plan``
    retains the existing SequenceResult contract for M3 robot validation.
    """
    cfg = config or QualitySearchConfig(); goal = goal_state or assembly.initial_state()
    if digest(grasp_analyzer.assembly) != digest(assembly): raise ValueError('Grasp analyzer belongs to another assembly')
    parts = {p.part_id:p for p in assembly.parts}
    if set(goal.poses) != set(parts): raise ValueError('Provide a full goal assembly state')
    movable = tuple(k for k,p in parts.items() if not p.fixed)
    base = AssemblyState({k:v for k,v in goal.poses.items() if parts[k].fixed}, goal.world_revision)
    supports = tuple(supports); ids = [s.candidate.support_id for s in supports]
    if len(ids) != len(set(ids)) or any(s.candidate.part_id not in parts for s in supports): raise ValueError('Invalid support IDs')
    if any(s.resource_id == cfg.sequence.handling.resource_id for s in supports): raise ValueError('Conflicting arm resources')
    oracle = SequenceEvaluator(assembly, cfg.sequence, supports)
    issues = []; rejects = []; metrics = {}; branches = {}; plans = {}
    binding = digest(('quality_sequence/1', assembly, goal, cfg, supports, grasp_analyzer.binding))

    def make_plan(steps):
        steps = tuple(steps); final_supports = steps[-1].supports_before
        key = digest(('sequence/1', assembly, goal, supports, final_supports, cfg.sequence))
        return SequenceResult('success', tuple(reversed(steps)), goal, base, supports, final_supports,
            cfg.sequence, key, dict(method='forward_quality_dfs', quality_binding=binding,
                                   elapsed_s=0., staging_and_robot_pending=True))

    def accept_complete(steps):
        if cfg.require_final_without_support and steps[-1].supports_before: return False
        plan = make_plan(steps); key = digest(tuple((s.part_id,s.supports_before,s.supports_after) for s in steps))
        replay = replay_sequence(assembly, plan, evaluator=oracle)
        if replay['status'] != 'valid': issues.append(dict(reason='complete_replay_failed', replay=replay)); return False
        plans[key] = replace(plan, geometry_validated=True, equilibrium_validated=True)
        return True

    def expand(context, pid):
        state, active = context
        after = AssemblyState(dict(state.poses, **{pid:goal.poses[pid]}), goal.world_revision)
        cache_key = digest((after, pid, active))
        if cache_key in branches:
            yield from branches[cache_key]; return
        branches[cache_key] = children = []
        grasp = grasp_analyzer.analyze(after, pid)
        if grasp.count is None:
            issues.append(dict(part_id=pid, reason='graspability_unknown', details=grasp)); return
        if grasp.count == 0:
            rejects.append(dict(part_id=pid, present=tuple(state.poses), reason='no_qualified_catalogue_grasp')); return
        graph = oracle.graph(after)
        directions = assembly_directions(graph, (pid,), config=cfg.direction)
        a = score_assemblability(directions)
        if a.score is None or directions.best_direction is None:
            target = rejects if a.score == 0 else issues
            target.append(dict(part_id=pid, reason='direction_'+directions.status)); return
        if a.score == 0: return
        state_key = digest(after)
        if state_key not in metrics:
            nominal = oracle.equilibrium(after, ())
            if _balanced(nominal):
                sweep = DirectionalStabilityAnalyzer(assembly, after, graph, config=cfg.sweep).analyze()
                value = sweep.sampled_minimum_load_factor
                metrics[state_key] = (value, dict(status=sweep.status, value=value,
                    scope='minimum_sampled_capped_radial_load_factor', sweep_digest=sweep.input_digest,
                    timings=sweep.diagnostics))
            elif nominal.status == 'infeasible': metrics[state_key] = (0., dict(status='nominal_infeasible'))
            else: metrics[state_key] = (None, dict(status=nominal.status, reason='nominal_unresolved'))
        s, evidence = metrics[state_key]
        if s is None: issues.append(dict(part_id=pid, reason='stability_unknown', details=evidence)); return
        s = 0. if s <= cfg.zero_stability_tol else float(s)
        if s > 0: subsets = [()]
        else:
            eligible = sorted(i for i,v in oracle.supports.items() if v.candidate.part_id in after.poses)
            choices = chain.from_iterable(combinations(eligible,n)
                for n in range(1,min(len(eligible),cfg.sequence.max_auxiliary_resources)+1))
            subsets = list(islice(choices,cfg.sequence.max_support_subsets+1))
            if len(subsets) > cfg.sequence.max_support_subsets:
                issues.append(dict(part_id=pid, reason='support_subset_budget'))
                subsets = subsets[:cfg.sequence.max_support_subsets]
        for chosen in subsets:
            if not oracle.resources_valid(tuple(sorted(set(chosen)|set(active)))): continue
            balance = oracle.equilibrium(after, chosen)
            if not _balanced(balance):
                if balance.status != 'infeasible': issues.append(dict(part_id=pid, reason='supported_equilibrium_unknown'))
                continue
            step, failure = oracle.evaluate(after, chosen, pid, remainder_support_ids=active,
                                           directions=np.asarray([directions.best_direction]))
            if step is None:
                # Bounded path searches and unqualified contacts cannot prove
                # global inassemblability or justify an optimality claim.
                issues.append(dict(part_id=pid, reason='transition_unresolved', details=failure)); continue
            step = replace(step, evidence=dict(step.evidence, graspability=grasp, assemblability=a,
                stability=evidence, quality_stability=s, insertion_direction=-directions.best_direction))
            child = QualityTransition((after, chosen), StepQuality(s, grasp.count, float(a.score)), step)
            children.append(child)
            yield child
        if not children: rejects.append(dict(part_id=pid, present=tuple(state.poses), reason='no_valid_support_transition'))

    raw = quality_depth_first(movable, expand, initial_context=(base, ()), config=cfg, accept_complete=accept_complete)
    best = raw.pop('best'); plan = None; qualities = ()
    if best is not None:
        _, qualities, steps = best
        key = digest(tuple((s.part_id,s.supports_before,s.supports_after) for s in steps))
        plan = plans[key]
        plan = replace(plan, diagnostics=dict(plan.diagnostics, elapsed_s=raw['elapsed_s'], quality=raw['score']))
    complete = raw['search_complete'] and not issues
    return QualitySequenceResult('success' if plan else 'unknown', plan, raw['score'], qualities,
        dict(raw, unresolved=issues, rejected=rejects, stability_states=len(metrics), transition_cache_size=len(branches),
             optimality='optimal_within_declared_catalogue_and_model' if complete and plan else 'not_proven',
             objective='Chen Eq.(2); finite ASP_OLD A; sampled normalized S', robot_execution_pending=True), binding, cfg)
