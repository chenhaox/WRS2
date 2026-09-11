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
    parser.add_argument('--backend',choices=['highs','numpy','cuda'],default='numpy')
    parser.add_argument('--mode',choices=['force','torque','wrench','legacy_coupled'],default='force')
    parser.add_argument('--directions',type=int,default=300)
    parser.add_argument('--max-score',type=float,default=100)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--port',type=int,default=8897)
    args=parser.parse_args()
    a,s,g=make_case(args.case)
    cfg=StabilitySweepConfig(backend=args.backend,mode=args.mode,direction_count=args.directions,max_score_n=args.max_score)
    analyzer=DirectionalStabilityAnalyzer(a,s,g,config=cfg,supports=case_supports(args.case))
    result=analyzer.analyze()
    print(result.status,'sampled minimum:',result.sampled_minimum_n,'N-equivalent',dict(result.diagnostics),flush=True)
    out=Path(__file__).parent/'output'/'stability_sweep'; out.mkdir(parents=True,exist_ok=True)
    save_report(result,out/f'{args.case}_{args.mode}_{args.backend}.json')
    if not args.headless:
        from _stability_sweep_display import show_sweep
        show_sweep(analyzer,result,port=args.port)


if __name__=='__main__': main()
