"""M2 sequence: choose a scene, set outside distance, then replay its checked path.

python examples/assembly/sequence_demo.py --case gantry --outside-mm 100 --headless
python examples/assembly/sequence_demo.py --case sleeve
"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from wrs.assembly import (SequenceConfig,MotionConfig,AuxiliarySupport,plan_sequence,replay_sequence,save_report)
from _sequence_cases import make_case,case_supports,DESCRIPTIONS


def compute_plan(case,method,outside_mm):
    assembly,state,graph=make_case(case)
    supports=tuple(AuxiliarySupport(f'aux_{i}',s) for i,s in enumerate(case_supports(case)))
    config=SequenceConfig(method=method,motion=MotionConfig(outside_margin_m=outside_mm/1000,
                                                          max_translation_step_m=.005))
    plan=plan_sequence(assembly,state,config=config,supports=supports,
                       initial_support_ids=tuple(s.candidate.support_id for s in supports))
    replay=replay_sequence(assembly,plan) if plan.status=='success' else {'status':'not_run'}
    output=Path(__file__).parent/'output'/'sequence'; output.mkdir(parents=True,exist_ok=True)
    save_report(plan,output/f'{case}_{method}_{outside_mm:g}mm.json')
    return assembly,state,graph,plan,replay


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=tuple(DESCRIPTIONS),default='gantry')
    parser.add_argument('--method',choices=('dfs','beam'),default='dfs')
    parser.add_argument('--outside-mm',type=float,default=100)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--port',type=int,default=8894)
    args=parser.parse_args()
    if not 10<=args.outside_mm<=300: parser.error('--outside-mm must be between 10 and 300')
    data=compute_plan(args.case,args.method,args.outside_mm)
    print('sequence:',data[3].status,dict(data[3].diagnostics),flush=True)
    print('independent forward replay:',data[4]['status'],flush=True)
    print('assembly order:',[s.part_id for s in data[3].assembly_steps],flush=True)
    if args.headless:
        if data[3].status!='success' or data[4]['status']!='valid': raise SystemExit(1)
        return
    from _sequence_display import show_sequence
    show_sequence(data,case=args.case,method=args.method,outside_mm=args.outside_mm,port=args.port)


if __name__=='__main__': main()
