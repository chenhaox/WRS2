"""Paper S/G/A quality DFS using WRS OR2FG7 grasps and existing M2 replay.

python examples/assembly/quality_sequence_demo.py --case bridge --headless
python examples/assembly/quality_sequence_demo.py --case counterweight --port 8898
"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
from wrs.assembly import (Assembly, AuxiliarySupport, SupportCandidate, GraspabilityAnalyzer,
    QualitySearchConfig, StabilitySweepConfig, plan_quality_sequence, replay_sequence,
    analyze_contacts, score_assemblability, solve_directions, save_report)
from _stability_cases import block


def make_quality_case(name):
    table = block('table',(.4,.25,.02),(0,0,-.01),fixed=True)
    if name == 'stack':
        parts = [table,block('lower',(.025,.025,.06),(0,0,.03),.1),
                 block('upper',(.025,.025,.06),(0,0,.09),.15)]
    elif name == 'bridge':
        parts = [table,block('left_pier',(.025,.06,.10),(-.075,0,.05),.2),
                 block('right_pier',(.025,.06,.10),(.075,0,.05),.2),
                 block('beam',(.23,.03,.03),(0,0,.115),.3),
                 block('payload',(.025,.025,.06),(.025,0,.16),.2)]
    elif name == 'counterweight':
        parts = [block('pedestal',(.04,.09,.1),(0,0,.05),fixed=True),
                 block('beam',(.24,.03,.03),(.08,0,.115),1.),
                 block('weight_lower',(.025,.025,.06),(-.025,0,.16),3.),
                 block('weight_upper',(.025,.025,.06),(-.025,0,.22),1.)]
    else: raise ValueError(name)
    supports = () if name != 'counterweight' else (AuxiliarySupport('aux_arm',
        SupportCandidate('beam_support','beam',(.16,0,.10),(0,0,1),20.)),)
    return Assembly(tuple(parts)), supports


def compute(case,backend,count,prune=True):
    from wrs import or_2fg7
    assembly,supports = make_quality_case(case); hand = or_2fg7.OR2FG7()
    grasp = GraspabilityAnalyzer(assembly,hand)
    cfg = QualitySearchConfig(prune=prune,sweep=StabilitySweepConfig(backend=backend,direction_count=count))
    result = plan_quality_sequence(assembly,grasp,supports=supports,config=cfg)
    output = Path(__file__).parent/'output'/'quality'; output.mkdir(parents=True,exist_ok=True)
    suffix = '' if prune else '_exhaustive'
    save_report(result,output/f'{case}_{backend}{suffix}.json')
    return assembly,grasp,hand,result


def show(assembly,analyzer,hand,result,port):
    from wrs import wvw
    from wrs.viewer.web_ui import Anchor
    from wrs.assembly.adapters.wrs_scene import scene_object_from_part
    from _contact_display import draw_contacts,contact_layers
    base = wvw.World(cam_pos=(.48,-.52,.42),cam_lookat_pos=(.03,0,.14),port=port)
    models = {p.part_id:scene_object_from_part(p) for p in assembly.parts}
    for model in models.values(): model.add_to_scene(base.scene)
    draw_contacts(base,contact_layers(analyze_contacts(assembly,assembly.initial_state()).patches),list(models.values()),
                  description='最终装配位置的接触面参考。抓取显示使用真实 OR2FG7。')
    hand.add_to_scene(base.scene)
    plan = result.plan; steps = plan.assembly_steps
    settings = dict(step=0,frame=0,grasp=0)
    panel = base.ui.add_panel('quality',title='S / G / A 评分与深度优先搜索',anchor=Anchor.TOP_RIGHT,width=370)
    panel.add_label('order',label='最佳已验证序列',value=' → '.join(s.part_id for s in steps))
    panel.add_label('score',label='序列综合分数',value=f'{result.score:.6g}；{result.diagnostics["elapsed_s"]:.3f} s')
    optimal = ('当前候选与模型内最优' if result.diagnostics['optimality']=='optimal_within_declared_catalogue_and_model'
               else '已有可行方案，最优性尚未证明')
    panel.add_label('search',label='搜索记录',value=f'{result.diagnostics["expansions"]} 次展开 / '
                    f'{result.diagnostics["pruned"]} 次上界剪枝；{optimal}')
    panel.add_label('scope',label='范围',value='S：归一化 6D 扰动；G：合格抓取数；A：方向集合类别评分。路径经过 M2 复核，机器人须继续 M3 验证。')
    panel.add_label('values',label='当前步骤评分')
    panel.add_label('support',label='当前支撑 / 到位后支撑')
    panel.add_label('grasp_count',label='抓取展示')
    panel.add_label('classes',label='A 类别表',value='面积 10；整圆 9；圆弧 3；双向轴 2；单向轴 1；锁死 0。与显示点数无关。')

    def draw():
        step = steps[settings['step']]; frames = step.assembly_poses
        index = min(settings['frame'],len(frames)-1); tf = frames[index]
        poses = dict(step.after.poses,**{step.part_id:tf})
        for pid,model in models.items():
            if pid in poses: model.tf=poses[pid]; base.scene.add(model)
            else: base.scene.remove(model)
        # The saved result contains immutable evidence; query returns the
        # analyzer's bound cached record, then exports fresh WRS Grasp objects.
        grasps = analyzer.accepted_grasps(analyzer.analyze(step.before,step.part_id))
        g = grasps[settings['grasp']%len(grasps)]; world = tf@g.pose
        hand.grip_at(world[:3,3],world[:3,:3],g.provenance['jaw_width'])
        q = result.qualities[settings['step']]; a = step.evidence['assemblability']
        panel.set_value('values',f'S={q.stability:.6g}；G={q.graspability}；A={q.assemblability:g} ({a["region"] if isinstance(a,dict) else a.region})')
        active = step.supports_before if index==len(frames)-1 else step.supports_after
        panel.set_value('support',f'{", ".join(active) or "无"} / {", ".join(step.supports_before) or "无"}')
        panel.set_value('grasp_count',f'{settings["grasp"]%len(grasps)+1}/{len(grasps)}；展示搬运姿态，夹爪沿途碰撞尚待 M3 检查')

    def frame(value):
        settings['frame']=min(max(int(value),0),len(steps[settings['step']].assembly_poses)-1)
        panel.set_value('frame',settings['frame']); draw()
    def change(pid):
        settings.update(step=[s.part_id for s in steps].index(pid),frame=0,grasp=0)
        panel.remove('frame')
        panel.add_slider('frame',label='M2 路径帧',min_value=0,max_value=max(1,len(steps[settings['step']].assembly_poses)-1),
                         step=1,value=0,on_change=frame)
        draw()
    def next_grasp(): settings['grasp']+=1; draw()
    panel.add_select('step',label='装配步骤',options=[s.part_id for s in steps],value=steps[0].part_id,on_change=change)
    panel.add_slider('frame',label='M2 路径帧',min_value=0,max_value=max(1,len(steps[0].assembly_poses)-1),step=1,value=0,on_change=frame)
    panel.add_button('next_grasp',label='下一个合格抓取',on_click=next_grasp)
    panel.add_button('arrive',label='查看本步到位',on_click=lambda:frame(len(steps[settings['step']].assembly_poses)-1))
    draw(); base.run()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=('stack','bridge','counterweight'),default='bridge')
    parser.add_argument('--backend',choices=('cuda','highs','numpy'),default='cuda')
    parser.add_argument('--directions',type=int,default=300)
    parser.add_argument('--no-prune',action='store_true',help='Compare the same finite model with exhaustive DFS')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--port',type=int,default=8898)
    args = parser.parse_args(); data = compute(args.case,args.backend,args.directions,not args.no_prune)
    a,g,h,r = data
    print('status:',r.status,'score:',r.score,'seconds:',r.diagnostics['elapsed_s'],flush=True)
    print('optimality:',r.diagnostics['optimality'],'unresolved:',len(r.diagnostics['unresolved']),flush=True)
    if r.plan is None:
        print(dict(r.diagnostics),flush=True); raise SystemExit(1)
    print('order:',[s.part_id for s in r.plan.assembly_steps],flush=True)
    print('S/G/A:',r.qualities,flush=True)
    print('forward replay:',replay_sequence(a,r.plan)['status'],flush=True)
    if not args.headless: show(*data,args.port)


if __name__=='__main__': main()
