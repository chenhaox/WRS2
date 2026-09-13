"""Plan a manifest in declared units; print M2 evidence without importing robots."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from wrs.assembly import load_assembly,save_report,plan_sequence,replay_sequence,SequenceConfig,MotionConfig


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('manifest',type=Path)
    p.add_argument('--output',type=Path,default=Path('assembly_plan.json'))
    p.add_argument('--method',choices=('dfs','beam'),default='dfs')
    p.add_argument('--clearance',type=float,default=.0002,help='non-contact clearance in metres')
    p.add_argument('--time-limit',type=float,default=120,help='cooperative search time budget in seconds')
    args=p.parse_args()
    assembly=load_assembly(args.manifest)
    config=SequenceConfig(method=args.method,time_limit_s=args.time_limit,
                          motion=MotionConfig(clearance_m=args.clearance))
    result=plan_sequence(assembly,config=config)
    save_report(result,args.output)
    print(result.status,dict(result.diagnostics),flush=True)
    if result.status!='success': raise SystemExit(1)
    print('assembly order:',[s.part_id for s in result.assembly_steps])
    print('independent replay:',replay_sequence(assembly,result)['status'])
    print('robot execution validated:',result.execution_validated)


if __name__=='__main__': main()
