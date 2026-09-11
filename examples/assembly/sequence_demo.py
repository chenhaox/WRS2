"""M2: finite object paths, support events and deterministic assembly sequence.

python examples/assembly/sequence_demo.py --headless
python examples/assembly/sequence_demo.py --case floating --method beam
"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from wrs.assembly import (SequenceConfig,AuxiliarySupport,plan_sequence,replay_sequence,save_report)
from wrs.assembly.adapters.wrs_scene import scene_object_from_part
from _stability_cases import make_case,case_supports


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=('stack','bridge','floating'),default='stack')
    parser.add_argument('--method',choices=('dfs','beam'),default='dfs')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--port',type=int,default=8894)
    args=parser.parse_args()
    assembly,state,graph=make_case(args.case)
    supports=tuple(AuxiliarySupport(f'aux_{i}',s) for i,s in enumerate(case_supports(args.case)))
    plan=plan_sequence(assembly,state,config=SequenceConfig(method=args.method),supports=supports,
                       initial_support_ids=tuple(s.candidate.support_id for s in supports))
    print('sequence:',plan.status,dict(plan.diagnostics),flush=True)
    if plan.status!='success': raise SystemExit(1)
    print('forward replay:',replay_sequence(assembly,plan)['status'],flush=True)
    for step in plan.assembly_steps:
        print(step.part_id,'->',[e['kind'] for e in step.assembly_events],flush=True)
    output=Path(__file__).parent/'output'/'sequence'; output.mkdir(parents=True,exist_ok=True)
    save_report(plan,output/f'{args.case}_{args.method}.json')
    if args.headless: return

    from wrs import wvw
    from wrs.viewer.web_ui import Anchor
    from _contact_display import contact_layers,draw_contacts
    base=wvw.World(cam_pos=(.65,-.85,.65),cam_lookat_pos=(0,0,.17),port=args.port)
    models={p.part_id:scene_object_from_part(p,collision=False) for p in assembly.parts}
    draw_contacts(base,contact_layers(p for e in graph.edges for p in e.patches),list(models.values()),
                  description='最终装配态接触面参考；M2 仅验证零件几何路径和平衡，机器人验证请运行 M3 例子。')
    frames=[]; poses=dict(plan.terminal_state.poses)
    for step in plan.assembly_steps:
        for tf in step.assembly_poses:
            frames.append((step.part_id,dict(poses,**{step.part_id:tf})))
        poses=dict(step.before.poses)
    settings={'frame':0,'playing':False}
    panel=base.ui.add_panel('sequence',title='M2 装配序列',anchor=Anchor.TOP_RIGHT,width=310)
    panel.add_label('order',value=' → '.join(s.part_id for s in plan.assembly_steps))
    panel.add_label('resources',value=f'主搬运资源 1；辅助资源 {len(supports)}；机器人执行尚未验证')
    panel.add_label('phase',value='')
    def show(index):
        index=int(index); settings['frame']=index
        pid,current=frames[index]
        for name,model in models.items():
            if name in current:
                model.tf=current[name]; base.scene.add(model)
            else: base.scene.remove(model)
        panel.set_value('frame',index); panel.set_value('phase',f'{index}/{len(frames)-1} · 安装 {pid}')
    def tick(dt):
        if settings['playing']:
            show(min(settings['frame']+1,len(frames)-1))
            if settings['frame']==len(frames)-1: settings['playing']=False
    def toggle(): settings['playing']=not settings['playing']
    panel.add_slider('frame',label='对象路径帧',min_value=0,max_value=len(frames)-1,step=1,value=0,on_change=show)
    panel.add_button('play',label='播放 / 暂停',on_click=toggle)
    panel.add_button('restart',label='回到开始',on_click=lambda:show(0))
    base.schedule_interval(tick,.1)
    show(0); base.run()


if __name__=='__main__': main()
