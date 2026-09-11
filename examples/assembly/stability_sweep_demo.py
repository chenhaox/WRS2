"""Directional load limits: one shared model, many independent disturbance LPs.

python examples/assembly/stability_sweep_demo.py --backend cuda --case bridge
python examples/assembly/stability_sweep_demo.py --mode legacy_coupled --headless
"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from wrs.assembly import StabilitySweepConfig,DirectionalStabilityAnalyzer,save_report
from _stability_cases import make_case,case_supports


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['stack','bridge','floating','tripod'],default='bridge')
    parser.add_argument('--backend',choices=['highs','numpy','cuda'],default='cuda')
    parser.add_argument('--mode',choices=['force','torque','wrench','legacy_coupled'],default='wrench')
    parser.add_argument('--directions',type=int,default=300)
    parser.add_argument('--max-score',type=float,default=100)
    parser.add_argument('--force-reference-n',type=float,default=10.,help='Task force scale F_ref, N')
    parser.add_argument('--torque-reference-nm',type=float,default=1.,help='Task torque scale T_ref, N m; L=T_ref/F_ref')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--port',type=int,default=8897)
    args=parser.parse_args()
    if args.force_reference_n<=0 or args.torque_reference_nm<=0: parser.error('task references must be positive')
    a,s,g=make_case(args.case)
    cfg=StabilitySweepConfig(backend=args.backend,mode=args.mode,direction_count=args.directions,max_score_n=args.max_score,
        force_reference_n=args.force_reference_n,torque_length_m=args.torque_reference_nm/args.force_reference_n)
    analyzer=DirectionalStabilityAnalyzer(a,s,g,config=cfg,supports=case_supports(args.case))
    result=analyzer.analyze()
    print(result.status,'sampled minimum:',result.sampled_minimum_n,'N-equivalent',
          'task load factor:',result.sampled_minimum_load_factor,dict(result.diagnostics),flush=True)
    out=Path(__file__).parent/'output'/'stability_sweep'; out.mkdir(parents=True,exist_ok=True)
    save_report(result,out/f'{args.case}_{args.mode}_{args.backend}.json')
    if not args.headless:
        from _stability_sweep_display import show_sweep
        show_sweep(analyzer,result,port=args.port)


if __name__=='__main__': main()
