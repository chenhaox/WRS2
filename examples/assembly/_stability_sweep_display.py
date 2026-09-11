"""Two 3D projections of normalized 6D loads; radii preserve F/torque ratio."""
import numpy as np
from wrs import wvw,wssop
from wrs.viewer.web_ui import Anchor
from wrs.assembly import limit_wrench,check_equilibrium
from wrs.assembly.adapters.wrs_scene import scene_object_from_part
from _contact_display import draw_contacts,contact_layers
from _stability_display import draw_stability


def show_sweep(analyzer,result,*,port):
    a,s,g=analyzer.assembly,analyzer.state,analyzer.graph
    base=wvw.World(cam_pos=(1.05,-1.15,.8),cam_lookat_pos=(.15,0,.22),port=port)
    models=[]
    for part in a.parts:
        obj=scene_object_from_part(part,collision=False); obj.tf=s.poses[part.part_id]
        base.scene.add(obj); models.append(obj)
    draw_contacts(base,contact_layers(p for e in g.edges for p in e.patches),models,
                  description='装配体旁两组投影：下方 F，上方 τ。点到中心的距离保留力／力矩比例；同色同编号是一项试验。')
    stability_panel=draw_stability(base,a,s,analyzer.nominal,analyzer.config.stability)
    # Put the existing force-site panel below the direction controls.
    stability_panel.configure(anchor=Anchor.BOTTOM_LEFT,width=310,height=310)
    panel=base.ui.add_panel('sweep',title='六维扰动承载极限',anchor=Anchor.TOP_RIGHT,width=365,height=690,movable=True)
    centers=(np.array([.36,0,.14]),np.array([.36,0,.43])); radius=.11; objects=[]
    # Three great circles outline each projection's unit ball, not a 6D sphere.
    angle=np.linspace(0,2*np.pi,120,endpoint=False)
    circle=np.column_stack((np.cos(angle),np.sin(angle),np.zeros_like(angle)))
    rings=np.vstack((circle,circle[:,[0,2,1]],circle[:,[2,0,1]]))
    for center in centers:
        base.scene.add(wssop.point_cloud(center+radius*rings,np.tile((.68,.72,.76),(len(rings),1))))
        for axis in range(3):
            color=np.eye(3)[axis]*.8+.1
            base.scene.add(wssop.arrow(center,center+radius*np.eye(3)[axis],shaft_radius=.0005,
                head_radius=.002,head_length=.005,rgb=color))
    settings=dict(body=analyzer.free[0],direction=0)
    panel.add_label('method',label='方法',value=f'{result.config.backend} / {result.config.mode}；{len(result.directions)} 方向/零件')
    panel.add_label('timing',label='计时',value=f'准备 {result.diagnostics["preparation_s"]:.3f}s；整批 {result.diagnostics["query_s"]:.3f}s；'
                    f'CPU 回退 {result.diagnostics.get("fallback_problems",0)} 个')
    panel.add_select('body',label='施加载荷的零件',options=list(analyzer.free),on_change=lambda v:change('body',v))
    panel.add_slider('direction',label='方向编号',min_value=0,max_value=len(result.directions)-1,step=1,
                     value=0,on_change=lambda v:change('direction',v))
    panel.add_button('worst',label='定位采样集合中最弱方向',on_click=lambda:worst())
    panel.add_label('components',label='归一化方向',value='')
    panel.add_label('selected',label='选中项',value='')
    panel.add_label('wrench',label='载荷',value='')
    panel.add_label('check',label='复核',value='')
    panel.add_label('legend',label='颜色',value='红→蓝：低→高承载；灰：达到查询上限；紫：未求解。每个点是独立载荷试验。')
    cfg=result.config
    panel.add_label('meaning',label='两组投影',value='下方 F、上方 τ；RGB 轴分别是 x/y/z。保留 6D 单位向量各半部的长度，不分别归一化。纯力矩在 F 球心；纯力在 τ 球心。')
    panel.add_label('metric',label='任务尺度',value=f'F_ref={cfg.force_reference_n:g} N，T_ref={cfg.force_reference_n*cfg.torque_length_m:g} N·m；L={cfg.torque_length_m:g} m。'
                    '固定 6D 方向最大化一个幅值 α；无量纲承载倍数 ρ=α/F_ref。' if cfg.mode!='legacy_coupled' else
                    '旧版对照：两幅值可独立变化，max(F+T/L)；不代表固定 6D 方向的承载。')
    panel.add_label('scope',label='范围',value='一次只扰动一个零件。采样最小值不保证所有六维方向，也不代表所有零件同时受扰。')

    def redraw():
        for obj in objects: base.scene.remove(obj)
        objects.clear()
        body=settings['body']; cases=[(i,c) for i,c in enumerate(result.cases) if c.part_id==body]
        if not cases: panel.set_value('selected','基础平衡未通过，无法计算方向极限。'); return
        scores=np.array([c.score_n or 0. for _,c in cases])
        high=max([c.score_n for _,c in cases if c.status=='optimal']+[1e-9])
        ratio=np.clip(scores/high,0,1)
        colors=np.column_stack((1-ratio,.2+.3*ratio,ratio))
        for j,(_,c) in enumerate(cases):
            if c.status=='limit_reached': colors[j]=(.6,.6,.6)
            elif c.status=='unknown': colors[j]=(.65,.2,.85)
        index,case=cases[settings['direction']]
        for center,offset in zip(centers,(0,3)):
            projection=result.directions[:,offset:offset+3]
            objects.append(wssop.point_cloud(center+radius*projection,colors))
            vector=projection[case.direction_index]
            point=center+radius*vector
            objects.append(wssop.sphere(point,radius=.004,rgb=(.1,.75,.3)))
            if np.linalg.norm(vector)>1e-12:
                objects.append(wssop.arrow(center,point,shaft_radius=.001,
                    head_radius=.004,head_length=.009,rgb=(.1,.75,.3)))
        d=result.directions[case.direction_index]
        panel.set_value('components',f'dF={np.round(d[:3],3).tolist()}，dT={np.round(d[3:],3).tolist()}；'
                        f'长度 {np.linalg.norm(d[:3]):.3f} / {np.linalg.norm(d[3:]):.3f}')
        if case.score_n is None:
            panel.set_value('selected','unknown'); panel.set_value('wrench',''); panel.set_value('check','未求解')
        else:
            load=limit_wrench(result,index)
            part=analyzer.parts[body]; tf=s.poses[body]; com=tf[:3,:3]@part.com_local_m+tf[:3,3]
            if np.linalg.norm(load.force_world_n)>0:
                objects.append(wssop.arrow(com,com+.09*load.force_world_n/np.linalg.norm(load.force_world_n),
                    shaft_radius=.0012,head_radius=.004,head_length=.012,rgb=(.8,.12,.8)))
            if np.linalg.norm(load.torque_world_nm)>0:
                objects.append(wssop.arrow(com,com+.09*load.torque_world_nm/np.linalg.norm(load.torque_world_nm),
                    shaft_radius=.0012,head_radius=.004,head_length=.012,rgb=(.1,.35,.95)))
            proof=check_equilibrium(a,s,g,config=result.config.stability,
                supports=analyzer.supports,external_wrenches=(*analyzer.external_wrenches,load))
            factor=f'；ρ={case.score_n/cfg.force_reference_n:.4f}' if cfg.mode!='legacy_coupled' else ''
            value_name='旧式评分' if cfg.mode=='legacy_coupled' else 'α'
            panel.set_value('selected',f'{body} 方向 {case.direction_index}：{case.status}；{value_name}={case.score_n:.4f} N 等效'+factor)
            panel.set_value('wrench',f'F={np.round(load.force_world_n,3).tolist()} N；τ={np.round(load.torque_world_nm,3).tolist()} N·m')
            panel.set_value('check',f'所选极限重新平衡：{proof.status}。质心处紫箭头为外力方向、蓝箭头为外力矩轴向，均固定长 90 mm。'
                            '左下蓝锥/绿反力是无额外扰动的 nominal 工况。')
        for obj in objects: base.scene.add(obj)

    def change(key,value): settings[key]=int(value) if key=='direction' else value; redraw()
    def worst():
        if result.worst_case_index is None: return
        case=result.cases[result.worst_case_index]; settings.update(body=case.part_id,direction=case.direction_index)
        panel.set_value('body',case.part_id); panel.set_value('direction',case.direction_index); redraw()
    worst() if result.worst_case_index is not None else redraw()
    base.run()
