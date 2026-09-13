"""Reproducible M2 and actual WRS replay timings. Optional two-arm run."""
import importlib.metadata
import platform
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import (SequenceConfig,SequenceEvaluator,plan_sequence,replay_sequence,
                          AuxiliarySupport,SupportCandidate,save_report,save_assembly)
from examples.assembly._shared.stability_cases import make_case,case_supports


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
REPEATS = 3
ROBOT = False
AUXILIARY = False
OUTPUT_DIR = ROOT / "benchmark_results" / "assembly" / "execution"


def main():
    if REPEATS<1: raise ValueError('REPEATS must be positive')
    output=OUTPUT_DIR
    report=dict(schema_version='wrs.assembly.execution_benchmark/1',python=sys.executable,
        platform=platform.platform(),processor=platform.processor(),
        packages={k:importlib.metadata.version(k) for k in ('numpy','scipy','mujoco','wgpu')},
        description='Timings include actual computation; export/viewer startup excluded. No timing pass threshold.',m2=[],m3=[])
    for name in ('stack','bridge','floating'):
        assembly,state,_=make_case(name); cfg=SequenceConfig()
        supports=tuple(AuxiliarySupport(f'aux_{i}',s) for i,s in enumerate(case_supports(name)))
        ids=tuple(s.candidate.support_id for s in supports)
        evaluator=SequenceEvaluator(assembly,cfg,supports)
        uncached=[]; reused=[]; replay=[]
        for i in range(REPEATS):
            start=perf_counter(); result=plan_sequence(assembly,state,config=cfg,supports=supports,initial_support_ids=ids)
            uncached.append(perf_counter()-start)
            start=perf_counter(); warm=plan_sequence(assembly,state,config=cfg,supports=supports,
                                                     initial_support_ids=ids,evaluator=evaluator)
            reused.append(perf_counter()-start)
            start=perf_counter(); replay_result=replay_sequence(assembly,result); replay.append(perf_counter()-start)
            assert result.status==warm.status=='success' and replay_result['status']=='valid'
        row=dict(case=name,triangles=sum(len(x.geometry.faces) for x in assembly.parts),
            fresh_evaluator_ms=np.array(uncached)*1000,reused_evaluator_ms=np.array(reused)*1000,
            independent_replay_ms=np.array(replay)*1000,expansions=result.diagnostics['expansions'])
        report['m2'].append(row)
        print(name,'fresh/reused/replay median ms:',*[round(float(np.median(x))*1000,3) for x in (uncached,reused,replay)],flush=True)
        if name=='stack': save_assembly(assembly,output/'stack_manifest.json')
    if ROBOT or AUXILIARY:
        from examples.assembly._shared.robot import make_demo
        from wrs.assembly.execution import validate_execution
        for auxiliary in ([False,True] if AUXILIARY else [False]):
            assembly,cell=make_demo(auxiliary)
            supports=() if not auxiliary else (AuxiliarySupport('aux_arm',SupportCandidate(
                'lower_support','lower',assembly.parts[1].assembled_tf[:3,3],(0,0,1),5.)),)
            plan=plan_sequence(assembly,supports=supports,initial_support_ids=() if not supports else ('lower_support',))
            start=perf_counter(); result=validate_execution(plan,cell); total=perf_counter()-start
            assert result.status=='success',dict(result.diagnostics)
            report['m3'].append(dict(auxiliary=auxiliary,total_s=total,diagnostics=result.diagnostics))
            save_report(result,output/('robot_execution_auxiliary.json' if auxiliary else 'robot_execution.json'))
            print('robot',auxiliary,total,dict(result.diagnostics),flush=True)
    save_report(report,output/'benchmark.json')


if __name__=='__main__': main()
