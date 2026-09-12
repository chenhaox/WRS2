"""Deterministic assembly-by-disassembly with explicit finite handling resources.

Motion is quasistatic. The remainder must stand independently of the carried
part throughout transfer; the carrier balances that part's declared loads.
This conservative decomposition supplies a whole-transfer equilibrium witness.
Staging/grasp feasibility remains an explicit obligation for robot execution.
"""
from dataclasses import dataclass, field, replace
from itertools import combinations
from time import perf_counter
import numpy as np
from .model import AssemblyState, freeze, digest
from .contact.analysis import analyze_contacts
from .contact.graph import build_contact_graph
from .geometry.proximity import MeshProximity
from .part_motion import MotionConfig, plan_removal, validate_object_path
from .stability import StabilityConfig, SupportCandidate, check_equilibrium


@dataclass(frozen=True)
class HandlingCapability:
    resource_id: str = 'main_arm'
    max_force_n: float = 100.
    max_torque_nm: float = 10.

    def __post_init__(self):
        if not self.resource_id: raise ValueError('Handling resource ID required')
        for k in ('max_force_n','max_torque_nm'):
            if not np.isfinite(getattr(self,k)) or getattr(self,k)<=0: raise ValueError('Positive finite handling capacities required')


@dataclass(frozen=True)
class AuxiliarySupport:
    resource_id: str
    candidate: SupportCandidate

    def __post_init__(self):
        if not self.resource_id or not isinstance(self.candidate,SupportCandidate):
            raise ValueError('A named resource and a finite SupportCandidate are required')


@dataclass(frozen=True)
class SequenceConfig:
    method: str = 'dfs'
    max_expansions: int = 64
    beam_width: int = 8
    max_auxiliary_resources: int = 2
    max_support_subsets: int = 32
    time_limit_s: float = 120.
    motion: MotionConfig = field(default_factory=MotionConfig)
    stability: StabilityConfig = field(default_factory=StabilityConfig)
    handling: HandlingCapability = field(default_factory=HandlingCapability)

    def __post_init__(self):
        if self.method not in ('dfs','beam'): raise ValueError('method must be dfs or beam')
        for k in ('max_expansions','beam_width','max_support_subsets'):
            if type(getattr(self,k)) is not int or getattr(self,k)<1: raise ValueError(k+' must be positive')
        if type(self.max_auxiliary_resources) is not int or self.max_auxiliary_resources<0:
            raise ValueError('Invalid auxiliary resource limit')
        if not np.isfinite(self.time_limit_s) or self.time_limit_s<=0: raise ValueError('Positive time limit required')


@dataclass(frozen=True,eq=False)
class SequenceStep:
    part_id: str
    before: AssemblyState
    after: AssemblyState
    removal: object
    supports_before: tuple[str,...]
    supports_after: tuple[str,...]
    events: tuple
    evidence: dict
    cost: float

    def __post_init__(self):
        for k in ('events','evidence'): object.__setattr__(self,k,freeze(getattr(self,k)))
        for k in ('supports_before','supports_after'): object.__setattr__(self,k,tuple(getattr(self,k)))

    @property
    def assembly_poses(self):
        return tuple(reversed(self.removal.poses))

    @property
    def assembly_events(self):
        inverse={'acquire_auxiliary':'release_auxiliary','release_auxiliary':'acquire_auxiliary',
                 'acquire_part':'release_part','move_to_outside':'insert_part','handoff_to_staging':'take_from_staging'}
        return tuple(freeze(dict(e,kind=inverse[e['kind']])) for e in reversed(self.events))


@dataclass(frozen=True,eq=False)
class SequenceResult:
    status: str
    removal_steps: tuple
    initial_state: AssemblyState
    terminal_state: AssemblyState
    supports: tuple[AuxiliarySupport,...]
    initial_support_ids: tuple
    config: SequenceConfig
    input_digest: str
    diagnostics: dict
    geometry_validated: bool = False
    equilibrium_validated: bool = False
    execution_validated: bool = False
    schema_version: str = 'wrs.assembly.sequence/1'

    def __post_init__(self):
        for k in ('removal_steps','supports','initial_support_ids'): object.__setattr__(self,k,tuple(getattr(self,k)))
        object.__setattr__(self,'diagnostics',freeze(self.diagnostics))

    @property
    def assembly_steps(self): return tuple(reversed(self.removal_steps))


def _restricted_config(config,state):
    return replace(config,disturbances=tuple(replace(c,wrenches=tuple(w for w in c.wrenches if w.part_id in state.poses))
                                             for c in config.disturbances))


def _balanced(result):
    return result.status=='feasible' and result.robustness_status in ('not_tested','passed_tested_set')


def _handling(assembly,part,config):
    if part.mass_kg is None or part.com_local_m is None: return None
    gravity=part.mass_kg*assembly.gravity_world_m_s2
    required=[(float(np.linalg.norm(gravity)),0.)]
    for case in config.stability.disturbances:
        loads=[w for w in case.wrenches if w.part_id==part.part_id]
        force=gravity+sum((w.force_world_n for w in loads),np.zeros(3))
        torque=sum((w.torque_world_nm for w in loads),np.zeros(3))
        required.append((float(np.linalg.norm(force)),float(np.linalg.norm(torque))))
    f,t=np.max(required,axis=0)
    return dict(feasible=bool(f<=config.handling.max_force_n and t<=config.handling.max_torque_nm),
                force_n=float(f),torque_at_com_nm=float(t),resource_id=config.handling.resource_id,
                meaning='finite_quasistatic_wrench_at_COM; grasp_and_staging_pending')


class SequenceEvaluator:
    """Instance-owned exact-state contact/equilibrium caches, no negative global cache."""
    def __init__(self,assembly,config,supports=()):
        self.assembly,self.config=assembly,config
        self.supports={s.candidate.support_id:s for s in supports}
        self.backend=MeshProximity(numerical_tol_m=config.motion.numerical_tol_m)
        self.graphs,self.balances={},{}

    def graph(self,state):
        key=digest(state)
        if key not in self.graphs:
            if len(self.graphs)>=128: self.graphs.clear()
            self.graphs[key]=build_contact_graph(self.assembly,state,analyze_contacts(self.assembly,state,backend=self.backend))
        return self.graphs[key]

    def equilibrium(self,state,ids):
        key=digest((state,tuple(sorted(ids))))
        if key not in self.balances:
            if len(self.balances)>=512: self.balances.clear()
            self.balances[key]=check_equilibrium(self.assembly,state,self.graph(state),
                config=_restricted_config(self.config.stability,state),
                supports=tuple(self.supports[i].candidate for i in ids))
        return self.balances[key]

    def resources_valid(self,ids):
        resources=[self.supports[i].resource_id for i in ids]
        return len(resources)==len(set(resources)) and len(resources)<=self.config.max_auxiliary_resources

    def evaluate(self,state,active,part_id,*,remainder_support_ids=None,directions=None):
        part=next(p for p in self.assembly.parts if p.part_id==part_id)
        handling=_handling(self.assembly,part,self.config)
        if handling is None or not handling['feasible']: return None,dict(reason='handling_capacity_or_mass_unknown',part_id=part_id)
        after=AssemblyState({k:v for k,v in state.poses.items() if k!=part_id},state.world_revision)
        candidates=sorted(i for i,s in self.supports.items() if s.candidate.part_id in after.poses)
        if remainder_support_ids is not None:
            remainder_support_ids=tuple(sorted(remainder_support_ids))
            if (len(remainder_support_ids)!=len(set(remainder_support_ids))
                    or not set(remainder_support_ids).issubset(candidates)):
                raise ValueError('Remainder supports must be distinct and act on present parts')
        checked=0
        sizes=(len(remainder_support_ids),) if remainder_support_ids is not None else range(min(len(candidates),self.config.max_auxiliary_resources)+1)
        for n in sizes:
            subsets=(remainder_support_ids,) if remainder_support_ids is not None else combinations(candidates,n)
            for chosen in subsets:
                checked+=1
                if checked>self.config.max_support_subsets:
                    return None,dict(reason='support_subset_budget',part_id=part_id)
                union=tuple(sorted(set(active)|set(chosen)))
                if not self.resources_valid(union): continue
                balance=self.equilibrium(after,chosen)
                if not _balanced(balance): continue
                # Acquiring new optional force generators preserves the previous
                # equilibrium; releasing old ones happens only after takeover.
                before=self.equilibrium(state,active)
                if not _balanced(before): return None,dict(reason='initial_equilibrium_unknown_or_failed',part_id=part_id)
                removal=plan_removal(self.assembly,state,part_id,graph=self.graph(state),backend=self.backend,
                                     config=self.config.motion,directions=directions)
                if removal.status!='success': return None,dict(reason='removal_search_exhausted',part_id=part_id,details=removal.diagnostics)
                events=[dict(kind='acquire_auxiliary',support_id=i,resource_id=self.supports[i].resource_id)
                        for i in chosen if i not in active]
                events.append(dict(kind='acquire_part',part_id=part_id,resource_id=self.config.handling.resource_id))
                events.extend(dict(kind='release_auxiliary',support_id=i,resource_id=self.supports[i].resource_id)
                              for i in active if i not in chosen)
                events.extend((dict(kind='move_to_outside',part_id=part_id),
                               dict(kind='handoff_to_staging',part_id=part_id,requires_execution_validation=True)))
                length=sum(np.linalg.norm(b[:3,3]-a[:3,3]) for a,b in zip(removal.poses,removal.poses[1:]))
                return SequenceStep(part_id,state,after,removal,active,chosen,tuple(events),
                    dict(before_equilibrium=before.input_digest,remainder_equilibrium=balance.input_digest,
                         handling=handling,transfer='independently_balanced_remainder_and_carried_part'),
                    float(length+.1*len(chosen))),None
        return None,dict(reason='remainder_needs_support_or_unqualified_contact',part_id=part_id)


def plan_sequence(assembly,initial_state=None,*,supports=(),initial_support_ids=(),config=None,evaluator=None):
    cfg=config or SequenceConfig(); state=initial_state or assembly.initial_state()
    supports=tuple(supports); initial_support_ids=tuple(sorted(initial_support_ids))
    ids=[s.candidate.support_id for s in supports]
    parts={p.part_id:p for p in assembly.parts}
    if len(ids)!=len(set(ids)) or not set(initial_support_ids).issubset(ids): raise ValueError('Invalid support IDs')
    if any(s.resource_id==cfg.handling.resource_id for s in supports): raise ValueError('Main and auxiliary resources must differ')
    if any(s.candidate.part_id not in parts for s in supports): raise ValueError('Support refers to unknown part')
    if any(k not in parts for k in state.poses): raise ValueError('State refers to unknown part')
    oracle=evaluator or SequenceEvaluator(assembly,cfg,supports)
    if digest((oracle.assembly,oracle.config,tuple(oracle.supports.values())))!=digest((assembly,cfg,supports)):
        raise ValueError('Evaluator is bound to another assembly/configuration/support set')
    if not oracle.resources_valid(initial_support_ids): raise ValueError('Initial resources exceed capacity or conflict')
    if any(oracle.supports[i].candidate.part_id not in state.poses for i in initial_support_ids):
        raise ValueError('Initial support must act on present part')
    key=digest(('sequence/1',assembly,state,supports,initial_support_ids,cfg))
    start=perf_counter(); failures=[]; expanded=0

    def finish(status,steps,terminal,reason):
        return SequenceResult(status,steps,state,terminal,supports,initial_support_ids,cfg,key,
            dict(reason=reason,elapsed_s=perf_counter()-start,expansions=expanded,failures=failures,
                 optimality='not_claimed',staging_and_robot_pending=True),status=='success',status=='success',False)

    if not _balanced(oracle.equilibrium(state,initial_support_ids)):
        return finish('unknown',(),state,'initial_equilibrium_unknown_or_failed')
    # DFS keeps untried siblings as a continuation. Do not calculate every
    # sibling's expensive geometry/equilibrium before following the first one.
    frontier=[(state,initial_support_ids,(),0.,None)]; seen=set()
    while frontier:
        current,active,steps,cost,pending=frontier.pop()
        remaining=[k for k in current.poses if not parts[k].fixed] if pending is None else pending
        if not remaining:
            plan=finish('success',steps,current,'candidate_found')
            replay=replay_sequence(assembly,plan,evaluator=oracle)
            if replay['status']=='valid': return plan
            failures.append(replay); return finish('unknown',(),state,'forward_replay_failed')
        if (pending is None and expanded>=cfg.max_expansions) or perf_counter()-start>cfg.time_limit_s:
            return finish('exhausted',(),state,'search_budget_exhausted')
        if pending is None:
            state_key=digest((current,active,tuple((i,oracle.supports[i].resource_id) for i in active)))
            if state_key in seen: continue
            seen.add(state_key); expanded+=1
            remaining.sort(key=lambda k:(-current.poses[k][2,3],k))
        children=[]
        for index,part_id in enumerate(remaining):
            if perf_counter()-start>cfg.time_limit_s: break
            step,failure=oracle.evaluate(current,active,part_id)
            if step is None: failures.append(failure); continue
            children.append((step.after,step.supports_after,steps+(step,),cost+step.cost,None))
            if cfg.method=='dfs':
                if index+1<len(remaining): frontier.append((current,active,steps,cost,remaining[index+1:]))
                break
        if cfg.method=='dfs': frontier.extend(children)
        else:
            frontier.extend(children)
            frontier.sort(key=lambda x:(len([k for k in x[0].poses if not parts[k].fixed]),x[3],tuple(x[0].poses)))
            frontier=list(reversed(frontier[:cfg.beam_width]))
    return finish('unknown',(),state,'finite_search_exhausted_not_global_infeasibility')


def replay_sequence(assembly,plan,*,evaluator=None):
    """Independently recheck forward insertion paths, state chain and load transfer."""
    if plan.status!='success': return dict(status='unknown',reason='no_complete_plan')
    oracle=evaluator or SequenceEvaluator(assembly,plan.config,plan.supports)
    if digest((oracle.assembly,oracle.config,tuple(oracle.supports.values())))!=digest((assembly,plan.config,plan.supports)):
        raise ValueError('Evaluator does not match this plan')
    expected=digest(('sequence/1',assembly,plan.initial_state,plan.supports,plan.initial_support_ids,plan.config))
    if expected!=plan.input_digest: raise ValueError('Plan is stale for this assembly/configuration')
    current=plan.terminal_state; active=plan.removal_steps[-1].supports_after if plan.removal_steps else plan.initial_support_ids
    checks=[]
    for step in plan.assembly_steps:
        expected_after=AssemblyState({k:v for k,v in step.before.poses.items() if k!=step.part_id},step.before.world_revision)
        if step.part_id not in step.before.poses or digest(expected_after)!=digest(step.after):
            return dict(status='unknown',reason='invalid_single_part_transition')
        expected_events=[dict(kind='acquire_auxiliary',support_id=i,resource_id=oracle.supports[i].resource_id)
                         for i in step.supports_after if i not in step.supports_before]
        expected_events.append(dict(kind='acquire_part',part_id=step.part_id,resource_id=plan.config.handling.resource_id))
        expected_events.extend(dict(kind='release_auxiliary',support_id=i,resource_id=oracle.supports[i].resource_id)
                               for i in step.supports_before if i not in step.supports_after)
        expected_events.extend((dict(kind='move_to_outside',part_id=step.part_id),
                                dict(kind='handoff_to_staging',part_id=step.part_id,requires_execution_validation=True)))
        if digest(tuple(expected_events))!=digest(step.events):
            return dict(status='unknown',reason='invalid_handoff_events')
        if digest(current)!=digest(step.after) or tuple(active)!=step.supports_after:
            return dict(status='unknown',reason='state_or_support_chain_mismatch')
        if not oracle.resources_valid(tuple(sorted(set(step.supports_before)|set(step.supports_after)))):
            return dict(status='unknown',reason='support_transition_resource_conflict')
        if not _balanced(oracle.equilibrium(current,step.supports_after)) or not _balanced(oracle.equilibrium(step.before,step.supports_before)):
            return dict(status='unknown',reason='forward_equilibrium_failed')
        part=next(p for p in assembly.parts if p.part_id==step.part_id)
        if part.fixed: return dict(status='unknown',reason='cannot_move_fixed_part')
        handling=_handling(assembly,part,plan.config)
        if handling is None or not handling['feasible']: return dict(status='unknown',reason='handling_capacity')
        outside=AssemblyState(dict(current.poses,**{step.part_id:step.assembly_poses[0]}),current.world_revision)
        validation=validate_object_path(assembly,outside,step.part_id,step.assembly_poses,
            policy=step.removal.policy,config=plan.config.motion,backend=oracle.backend)
        if validation.status!='valid': return dict(status='unknown',reason='forward_geometry_failed',part_id=step.part_id,details=validation)
        if not np.allclose(step.assembly_poses[-1],step.before.poses[step.part_id],atol=1e-9,rtol=0):
            return dict(status='unknown',reason='target_pose_mismatch')
        checks.append(dict(part_id=step.part_id,geometry=validation.status,equilibrium='feasible'))
        current,active=step.before,step.supports_before
    if digest(current)!=digest(plan.initial_state) or tuple(active)!=tuple(plan.initial_support_ids):
        return dict(status='unknown',reason='incomplete_forward_plan')
    return dict(status='valid',checks=checks,execution_validated=False)


plan_assembly=plan_sequence
