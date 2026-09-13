"""Small WRS controls; planning runs in one worker, drawing stays on the main loop."""
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from wrs import wvw
from wrs.viewer.web_ui import Anchor
from wrs.assembly.adapters.wrs_scene import scene_object_from_part
from .contact_display import contact_layers,draw_contacts
from .sequence_cases import DESCRIPTIONS


def path_distances(poses):
    """Translation arc length in mm at each *validated* path pose."""
    xyz=np.array([tf[:3,3] for tf in poses])
    return np.r_[0,np.cumsum(np.linalg.norm(np.diff(xyz,axis=0),axis=1))]*1000


def show_sequence(data,*,case,method,outside_mm,port):
    from .sequence import compute_plan
    base=wvw.World(cam_pos=(.72,-.85,.62),cam_lookat_pos=(0,0,.15),port=port)
    executor=ThreadPoolExecutor(max_workers=1)
    settings=dict(case=case,distance=outside_mm,playing=False,step=0,frame=0,pending=None)
    models={}; overlays=[]; current={'data':data}; dynamic=[]
    panel=base.ui.add_panel('sequence',title='M2 装配序列',anchor=Anchor.TOP_RIGHT,
                           width=335,movable=True,description='改变离开距离后重新验证路径；播放只使用已验证的姿态。')
    panel.add_select('case',label='例子',options=list(DESCRIPTIONS),value=case,
                     on_change=lambda v:settings.update(case=v))
    panel.add_slider('outside',label='外部位置额外离开距离',unit='mm',min_value=10,max_value=300,
                     step=1,value=outside_mm,on_change=lambda v:settings.update(distance=v))
    panel.add_label('distance_hint',label='距离含义',value='该值是超过其余零件在移动方向上的投影边界后的余量。总路程还包含退出装配体的距离。')
    panel.add_label('status',label='验证结果',value='')
    panel.add_label('description',label='场景',value='')
    panel.add_label('order',label='装配顺序',value='')
    panel.add_label('phase',label='当前动作',value='')
    panel.add_label('resources',label='搬运与支撑',value='')
    panel.add_label('verified',label='验证范围',value='M2 验证几何与静力平衡；机器人执行请运行 M3。')

    def show_frame(frame):
        plan=current['data'][3]
        if plan.status!='success': return
        step_index=settings['step']; step=plan.assembly_steps[step_index]
        frame=min(max(int(frame),0),len(step.assembly_poses)-1); settings['frame']=frame
        poses=dict(step.after.poses); poses[step.part_id]=step.assembly_poses[frame]
        for pid,model in models.items():
            if pid in poses: model.tf=poses[pid]; base.scene.add(model)
            else: base.scene.remove(model)
        distances=path_distances(step.assembly_poses)
        panel.set_value('frame',frame); panel.set_value('travel',round(float(distances[frame]),3))
        panel.set_value('phase',f'{step_index+1}/{len(plan.assembly_steps)} 安装 {step.part_id}：'
                        f'{distances[frame]:.1f} / {distances[-1]:.1f} mm；帧 {frame}/{len(distances)-1}')
        active=step.supports_before if frame==len(distances)-1 else step.supports_after
        panel.set_value('resources',f'主搬运资源 1；辅助候选 {len(plan.supports)}；'
                        f'当前平衡支撑 {", ".join(active) or "无"}。主/辅助机器人尚需 M3 验证。')

    def change_step(value):
        plan=current['data'][3]; settings['step']=[s.part_id for s in plan.assembly_steps].index(value)
        for key in ('travel','frame'):
            if key in dynamic: panel.remove(key); dynamic.remove(key)
        poses=plan.assembly_steps[settings['step']].assembly_poses; distances=path_distances(poses)
        panel.add_slider('travel',label='本步已移动距离（就近路径帧）',unit='mm',min_value=0,
                         max_value=max(.001,round(float(distances[-1]),3)),step=.001,value=0,
                         on_change=lambda mm:show_frame(np.argmin(abs(distances-mm))))
        panel.add_slider('frame',label='路径帧（含旋转）',min_value=0,max_value=max(1,len(poses)-1),step=1,
                         value=0,on_change=show_frame)
        dynamic.extend(('travel','frame')); panel.set_value('step',value); show_frame(0)

    def install(new_data):
        current['data']=new_data; assembly,state,graph,plan,replay=new_data
        for obj in [*models.values(),*overlays]: base.scene.remove(obj)
        if models: base.ui.remove_panel('contacts')
        models.clear(); overlays.clear()
        for part in assembly.parts:
            models[part.part_id]=scene_object_from_part(part,collision=False)
            models[part.part_id].tf=state.poses[part.part_id]; base.scene.add(models[part.part_id])
        overlays.extend(draw_contacts(base,contact_layers(p for e in graph.edges for p in e.patches),
            list(models.values()),description='最终装配位置的接触面参考。运动中的零件尚未到位时，这些参考面不是当前接触。'))
        for key in dynamic: panel.remove(key)
        dynamic.clear(); settings.update(step=0,frame=0,playing=False)
        panel.set_value('description',DESCRIPTIONS[settings['case']])
        panel.set_value('status',f'{plan.status}；复核 {replay["status"]}；规划 {plan.diagnostics["elapsed_s"]:.2f} s；'
                        f'已应用余量 {plan.config.motion.outside_margin_m*1000:g} mm')
        panel.set_value('order',' → '.join(s.part_id for s in plan.assembly_steps))
        if plan.status=='success' and replay['status']=='valid':
            panel.add_select('step',label='装配步骤',options=[s.part_id for s in plan.assembly_steps],on_change=change_step)
            dynamic.append('step'); change_step(plan.assembly_steps[0].part_id)
        else: panel.set_value('phase','没有通过验证的可播放序列。')

    def request_plan():
        if settings['pending'] is not None: return
        settings['playing']=False
        panel.set_value('status','正在后台规划与复核……')
        for key in ('case','outside','replan','play','restart',*dynamic): panel.set_enabled(key,False)
        settings['pending']=executor.submit(compute_plan,settings['case'],method,settings['distance'])

    def tick(dt):
        future=settings['pending']
        if future is not None and future.done():
            settings['pending']=None
            try: new_data=future.result()
            except Exception as error: panel.set_value('status',f'重新规划失败：{error}；保留之前的结果。')
            else: install(new_data)
            for key in ('case','outside','replan','play','restart',*dynamic): panel.set_enabled(key,True)
        if settings['playing'] and 'step' in dynamic:
            plan=current['data'][3]; step=plan.assembly_steps[settings['step']]
            if settings['frame']+1<len(step.assembly_poses): show_frame(settings['frame']+1)
            elif settings['step']+1<len(plan.assembly_steps): change_step(plan.assembly_steps[settings['step']+1].part_id)
            else: settings['playing']=False

    def toggle_play():
        plan=current['data'][3]
        if not settings['playing'] and 'step' in dynamic:
            last=plan.assembly_steps[-1]
            if settings['step']==len(plan.assembly_steps)-1 and settings['frame']==len(last.assembly_poses)-1:
                change_step(plan.assembly_steps[0].part_id)
        settings['playing']=not settings['playing']

    panel.add_button('replan',label='应用距离 / 例子并重新规划',on_click=request_plan)
    panel.add_button('play',label='播放 / 暂停',on_click=toggle_play)
    panel.add_button('restart',label='本步回到开始',on_click=lambda:show_frame(0))
    install(data); base.schedule_interval(tick,.1)
    try: base.run()
    finally: executor.shutdown(wait=False,cancel_futures=True)
