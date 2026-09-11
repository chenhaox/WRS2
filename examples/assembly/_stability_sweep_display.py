"""Direction sphere and the selected actual load in a WRS scene."""
import numpy as np
from wrs import wvw,wssop
from wrs.viewer.web_ui import Anchor
from wrs.assembly import limit_wrench,check_equilibrium
from wrs.assembly.adapters.wrs_scene import scene_object_from_part
from _contact_display import draw_contacts,contact_layers
from _stability_display import draw_stability


def show_sweep(analyzer,result,*,port):
    a,s,g=analyzer.assembly,analyzer.state,analyzer.graph
    base=wvw.World(cam_pos=(.9,-.9,.65),cam_lookat_pos=(.16,0,.15),port=port)
    models=[]
    for part in a.parts:
        obj=scene_object_from_part(part,collision=False); obj.tf=s.poses[part.part_id]
        base.scene.add(obj); models.append(obj)
    draw_contacts(base,contact_layers(p for e in g.edges for p in e.patches),models,
                  description='左：装配体。右：各方向的极限承载；球点位置表示方向，颜色表示承载大小。')
    stability_panel=draw_stability(base,a,s,analyzer.nominal,analyzer.config.stability)
    # Put the existing force-site panel below the direction controls.
    stability_panel.configure(anchor=Anchor.BOTTOM_LEFT,width=310,height=310)
    panel=base.ui.add_panel('sweep',title='多方向扰动承载极限',anchor=Anchor.TOP_RIGHT,width=350,movable=True)
    center=np.array([.35,0,.18]); radius=.11; objects=[]
    settings=dict(body=analyzer.free[0],direction=0)
    panel.add_label('method',label='方法',value=f'{result.config.backend} / {result.config.mode}；{len(result.directions)} 方向/零件')
    panel.add_label('timing',label='计时',value=f'准备 {result.diagnostics["preparation_s"]:.3f}s；整批 {result.diagnostics["query_s"]:.3f}s；'
                    f'CPU 回退 {result.diagnostics.get("fallback_problems",0)} 个')
    panel.add_label('legend',label='颜色',value='红→蓝：低→高承载；灰：达到查询上限；紫：未求解。每个点是独立载荷试验。')
    panel.add_label('meaning',label='方向含义',value=f'力矩按 {result.config.torque_length_m:g} m 换算为 N 等效评分；混合模式球面只显示力方向，力矩方向见数值。不是旋转装配体。')
    panel.add_label('selected',label='选中项',value='')
    panel.add_label('wrench',label='载荷',value='')
    panel.add_label('check',label='复核',value='')

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
        directions=result.directions[:,:3] if result.config.mode!='torque' else result.directions[:,3:]
        norms=np.linalg.norm(directions,axis=1); sphere=directions/np.maximum(norms[:,None],1e-12)
        objects.append(wssop.point_cloud(center+radius*sphere,colors))
        index,case=cases[settings['direction']]
        objects.append(wssop.arrow(center,center+radius*sphere[case.direction_index],shaft_radius=.001,
                                    head_radius=.004,head_length=.009,rgb=(.1,.75,.3)))
        if case.score_n is None:
            panel.set_value('selected','unknown'); panel.set_value('wrench',''); panel.set_value('check','未求解')
        else:
            load=limit_wrench(result,index)
            part=analyzer.parts[body]; tf=s.poses[body]; com=tf[:3,:3]@part.com_local_m+tf[:3,3]
            if np.linalg.norm(load.force_world_n)>0:
                objects.append(wssop.arrow(com,com+.09*load.force_world_n/np.linalg.norm(load.force_world_n),
                    shaft_radius=.0012,head_radius=.004,head_length=.012,rgb=(.8,.12,.8)))
            proof=check_equilibrium(a,s,g,config=result.config.stability,
                supports=analyzer.supports,external_wrenches=(*analyzer.external_wrenches,load))
            panel.set_value('selected',f'{body} 方向 {case.direction_index}：{case.status}；{case.score_n:.4f} N 等效')
            panel.set_value('wrench',f'F={np.round(load.force_world_n,3).tolist()} N；τ={np.round(load.torque_world_nm,3).tolist()} N·m')
            panel.set_value('check',f'所选极限重新平衡：{proof.status}。紫箭头仅表示所选外力方向，固定长 90 mm。'
                            '左下蓝锥/绿反力是无额外扰动的 nominal 工况。')
        for obj in objects: base.scene.add(obj)

    def change(key,value): settings[key]=int(value) if key=='direction' else value; redraw()
    panel.add_select('body',label='施加载荷的零件',options=list(analyzer.free),on_change=lambda v:change('body',v))
    panel.add_slider('direction',label='方向编号',min_value=0,max_value=len(result.directions)-1,step=1,
                     value=0,on_change=lambda v:change('direction',v))
    def worst():
        if result.worst_case_index is None: return
        case=result.cases[result.worst_case_index]; settings.update(body=case.part_id,direction=case.direction_index)
        panel.set_value('body',case.part_id); panel.set_value('direction',case.direction_index); redraw()
    panel.add_button('worst',label='定位采样集合中最弱方向',on_click=worst)
    worst() if result.worst_case_index is not None else redraw()
    base.run()
