"""State-explicit M2 planning and independent replay for the viewer."""
from wrs.assembly import (SequenceConfig,MotionConfig,AuxiliarySupport,plan_sequence,replay_sequence)
from .sequence_cases import make_case,case_supports,DESCRIPTIONS


def compute_plan(case,method,outside_mm):
    assembly,state,graph=make_case(case)
    supports=tuple(AuxiliarySupport(f'aux_{i}',s) for i,s in enumerate(case_supports(case)))
    config=SequenceConfig(method=method,motion=MotionConfig(outside_margin_m=outside_mm/1000,
                                                          max_translation_step_m=.005))
    plan=plan_sequence(assembly,state,config=config,supports=supports,
                       initial_support_ids=tuple(s.candidate.support_id for s in supports))
    replay=replay_sequence(assembly,plan) if plan.status=='success' else {'status':'not_run'}
    return assembly,state,graph,plan,replay
