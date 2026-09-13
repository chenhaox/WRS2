"""Declared single/dual-arm workcell and verified WRS execution replay."""
import numpy as np
from wrs.assembly import Assembly,Part
from wrs.assembly.geometry.primitives import box,pose
from wrs.assembly.planning.sequence import plan_sequence,SequenceConfig
from wrs.assembly.planning.sequence import AuxiliarySupport
from wrs.assembly import SupportCandidate
from wrs.assembly.robotics.execution import ExecutionArm,ExecutionWorkcell,ExecutionConfig,validate_execution


def make_demo(auxiliary=False):
    from wrs import khi_rs007l,or_2fg7
    robot=khi_rs007l.RS007L(); gripper=or_2fg7.OR2FG7()
    robot.mount(gripper,robot.runtime_lnks[-1],update=True)
    height=.18 if not auxiliary else .38
    a=Assembly((
        Part('table',box((.64,.60,.03)),pose((.47,0,.135)),fixed=True,friction=.5),
        Part('lower',box((.025,.025,.06)),pose((.50,0,height)),mass_kg=.10,com_local_m=(0,0,0),friction=.5),
        Part('upper',box((.025,.025,.06)),pose((.50,0,height+.06)),mass_kg=.15,com_local_m=(0,0,0),friction=.5)))
    sources={'lower':pose((.30,-.15,.18)),'upper':pose((.30,.15,.18))}
    arms={'main_arm':ExecutionArm(robot,finger_normal_force_n=30.,contact_friction=.5)}
    grasps={}
    if auxiliary:
        from scipy.spatial.transform import Rotation
        from wrs.grasp.grasp import Grasp
        helper=khi_rs007l.RS007L(); hand=or_2fg7.OR2FG7()
        helper.pos=(1.15,0,0); helper.rotmat=Rotation.from_euler('z',np.pi).as_matrix()
        helper.mount(hand,helper.runtime_lnks[-1],update=True)
        arms['aux_arm']=ExecutionArm(helper,finger_normal_force_n=30.,contact_friction=.5)
        gpose=pose((0,0,-.018),Rotation.from_euler('y',-np.pi/2).as_matrix())
        pre=gpose.copy(); pre[:3,3]-=gpose[:3,2]*.06
        grasps[('aux_arm','lower')]=[Grasp.from_jaw(hand,gpose,pre,.025,float(hand.jaw_range[1]))]
    cell=ExecutionWorkcell(a,arms,sources,grasps=grasps)
    return a,cell


def compute(auxiliary=False):
    a,cell=make_demo(auxiliary)
    supports=() if not auxiliary else (AuxiliarySupport('aux_arm',
        SupportCandidate('lower_support','lower',a.parts[1].assembled_tf[:3,3],(0,0,1),5.)),)
    plan=plan_sequence(a,config=SequenceConfig(),supports=supports,
                       initial_support_ids=() if not supports else ('lower_support',))
    print('M2:',plan.status,[s.part_id for s in plan.assembly_steps],flush=True)
    result=validate_execution(plan,cell,config=ExecutionConfig())
    print('M3:',result.status,dict(result.diagnostics),flush=True)
    if result.status!='success': raise SystemExit(1)
    return a,cell,result


def show(a,cell,result,port=8895):
    from wrs import wvw
    from wrs.viewer.web_ui import Anchor
    from .contact_display import contact_layers,draw_contacts
    from wrs.assembly import analyze_contacts
    base=wvw.World(cam_pos=(2,-2.4,1.9),cam_lookat_pos=(.5,0,.65),port=port)
    for b in cell.bindings.values(): b.arm.body.add_to_scene(base.scene)
    for obj in cell.objects.values(): obj.add_to_scene(base.scene)
    draw_contacts(base,contact_layers(analyze_contacts(a,a.initial_state()).patches),list(cell.objects.values()),
                  description='最终装配态接触面的参考位置；滑条显示实际机器人与零件运动。')
    settings={'frame':0,'playing':False}
    panel=base.ui.add_panel('execution',title='机器人装配回放',anchor=Anchor.TOP_RIGHT,width=300)
    panel.add_label('result',label='验证结果',value=f'{result.status}: {len(result.frames)} 帧；RS007L + OR2FG7')
    panel.add_label('meaning',label='范围',value='采样运动学与碰撞验证；未连接实机，未做动态仿真。')
    panel.add_label('time',label='规划及独立复核',value=f'{result.diagnostics["elapsed_s"]:.2f} 秒')
    def show(index):
        index=int(index)
        settings['frame']=int(index); frame=result.frames[int(index)]
        for rid,b in cell.bindings.items():
            b.arm.body.fk(qs=frame['robot_qs'][rid]); b.arm.end_effector.fk(qs=frame['gripper_qs'][rid])
        for pid,tf in frame['object_poses'].items(): cell.objects[pid].tf=tf
        panel.set_value('phase',f'{index}/{len(result.frames)-1} · {frame["target"]} · {frame["phase"]}')
        panel.set_value('frame',index)
    panel.add_label('phase',label='当前阶段')
    panel.add_slider('frame',label='轨迹帧',min_value=0,max_value=len(result.frames)-1,step=1,value=0,on_change=show)
    panel.add_button('next',label='下一帧',on_click=lambda:show(min(len(result.frames)-1,settings['frame']+1)))
    def play_pause(): settings['playing']=not settings['playing']
    def tick(dt):
        if settings['playing']:
            show(min(len(result.frames)-1,settings['frame']+4))
            if settings['frame']==len(result.frames)-1: settings['playing']=False
    panel.add_button('play',label='播放 / 暂停',on_click=play_pause)
    panel.add_button('restart',label='回到开始',on_click=lambda:show(0))
    panel.add_button('last',label='查看装配完成',on_click=lambda:show(len(result.frames)-1))
    base.schedule_interval(tick,.05)
    show(0); base.run()
