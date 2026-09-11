"""Draw exactly the LP's force sites, cone generators and returned forces."""
import numpy as np
from wrs import wssop
from wrs.viewer.web_ui import Anchor


def draw_stability(base, assembly, state, result, config):
    free = {p.part_id:p for p in assembly.parts if not p.fixed}
    cases = {c.name:c for c in (result.nominal,*result.disturbances) if c is not None}
    load_cases = {c.name:c.wrenches for c in config.disturbances}
    objects = {'points':[], 'cones':[], 'forces':[]}
    settings = dict(body='全部自由零件', load='nominal', points=True, cones=True, forces=True)
    panel = base.ui.add_panel('stability',title='静力平衡',anchor=Anchor.TOP_RIGHT,
                               width=330,offset=16,font_size=12,movable=True,
                               description='黄点：接触力点。青点：额外辅助支撑。蓝锥：允许施力方向。绿箭头：求得的力。')
    cone_counts = dict(cones=0,rays=0)

    def layer_status():
        if not settings['cones']:
            text = '摩擦锥已隐藏，可点击下方“恢复受力图层”。'
        elif not sum(cone_counts.values()):
            text = '所选零件没有可用力点。'
        else:
            text = f'显示 {cone_counts["cones"]} 个摩擦锥、{cone_counts["rays"]} 条无摩擦射线。'
        panel.set_value('layer_status',text)

    def add_arrow(start, vector, rgb):
        if np.linalg.norm(vector) > 1e-10:
            objects['forces'].append(wssop.arrow(start,start+.003*vector,
                shaft_radius=.0007,head_radius=.002,head_length=.006,rgb=rgb))

    def redraw():
        for group in objects.values():
            for obj in group:
                base.scene.remove(obj)
            group.clear()
        selected = set(free) if settings['body']=='全部自由零件' else {settings['body']}
        cone_vertices, cone_faces = [], []
        site_count = support_count = 0
        cone_counts.update(cones=0,rays=0)
        for site in result.force_sites:
            targets = [(pid,sign) for pid,sign in ((site['part_a'],-1),(site['part_b'],1)) if pid in selected]
            if not targets:
                continue
            site_count += 1
            is_support = site['kind']=='support'
            support_count += int(is_support)
            p = site['point_world_m']
            objects['points'].append(wssop.icosphere(pos=p,radius=.0022 if is_support else .0018,
                rgb=(0.,.75,.95) if is_support else (1.,.7,.05),subdivisions=1))
            for _,sign in targets:
                # Fixed 25 mm axial height, not a force magnitude or capacity.
                rim = p + .025*sign*site['rays_on_b_world']
                if len(rim) == 1: # frictionless: cone collapses to a ray
                    cone_counts['rays'] += 1
                    objects['cones'].append(wssop.linsegs([[p,rim[0]]],radius=.0004,
                                                          srgbs=np.array([.1,.45,1.])))
                    continue
                cone_counts['cones'] += 1
                start = len(cone_vertices)
                cone_vertices.extend([p,*rim])
                cone_faces.extend((start,start+1+i,start+1+(i+1)%len(rim)) for i in range(len(rim)))
        if cone_faces:
            # Batched mesh: no Python SceneObject for each of the 16 cone edges.
            for fs in (np.array(cone_faces),np.array(cone_faces)[:,::-1]):
                objects['cones'].append(wssop.mesh(cone_vertices,fs,rgb=(.12,.42,.95),alpha=.35))
        case = cases.get(settings['load'])
        if case is not None and case.status=='feasible':
            for force in (*case.contact_forces,*case.support_forces):
                p = force['point_world_m']
                if force['part_b'] in selected:
                    add_arrow(p,force['force_on_b_world_n'],(.02,.65,.3))
                if force.get('part_a') in selected:
                    add_arrow(p,force['force_on_a_world_n'],(.02,.65,.3))
        for pid in sorted(selected):
            part, tf = free[pid], state.poses[pid]
            com = tf[:3,:3] @ part.com_local_m + tf[:3,3]
            add_arrow(com,part.mass_kg*assembly.gravity_world_m_s2,(.88,.15,.12))
            for wrench in load_cases.get(settings['load'],()):
                if wrench.part_id == pid:
                    add_arrow(com,wrench.force_world_n,(.8,.12,.8))
        for key,group in objects.items():
            if settings[key]:
                for obj in group:
                    base.scene.add(obj)
        status = case.status if case else 'unknown'
        if result.issues:
            status = f'unknown（子模型 {status}）'
        panel.set_value('status',status + ('；没有可行反力解' if not case or case.status!='feasible' else ''))
        panel.set_value('points_count',f'所选：{site_count-support_count} 个接触力点 + {support_count} 个辅助点')
        layer_status()
        panel.set_value('body_info','；'.join(f'{pid}: {free[pid].mass_kg:g} kg, μ={free[pid].friction:g}' for pid in sorted(selected)))
        torques = [f'{w.part_id}: {np.round(w.torque_world_nm,3).tolist()} N·m'
                   for w in load_cases.get(settings['load'],()) if np.linalg.norm(w.torque_world_nm)>0]
        panel.set_value('torque','；'.join(torques) or '无额外外力矩')
        residual = max((np.max(np.abs(r['force_world_n'])) for r in case.body_residuals.values()),default=0) if case else None
        panel.set_value('residual',f'{residual:.2e} N' if case and case.status=='feasible' else '未求得可行解')

    def change(key,value):
        settings[key] = value
        if key in objects:
            for obj in objects[key]:
                (base.scene.add if value else base.scene.remove)(obj)
            layer_status()
            return
        redraw()

    panel.add_select('body',label='观察受力零件',options=['全部自由零件',*sorted(free)],
                     on_change=lambda v:change('body',v))
    panel.add_select('load',label='载荷工况',options=list(cases) or ['nominal'],
                     on_change=lambda v:change('load',v))
    panel.add_label('status',label='当前工况')
    panel.add_label('robust',label='扰动集合',value=result.robustness_status)
    panel.add_label('points_count',label='参与计算的点')
    clean_count = result.diagnostics.get('clean_contact_points',0)
    contact_count = sum(site['kind']=='contact' for site in result.force_sites)
    panel.add_label('reduction',label='整套接触力点',value=f'清理后 {clean_count} → 求解 {contact_count}；'
                    +('约简开启' if config.reduce_contact_points else '约简关闭'))
    auxiliary = [site for site in result.force_sites if site['kind']=='support']
    if auxiliary:
        panel.add_label('auxiliary',label='声明的辅助支撑',value='；'.join(
            f'{site["support_id"]}: ≤{site["max_group_normal_force_n"]:g} N, μ={site["friction"]:g}' for site in auxiliary)
            +'。μ=0 的锥退化为射线；不是检测到的接触面。')
    if result.diagnostics.get('curved_fallback'):
        panel.add_label('fallback',label='曲面采样',value='子集未通过全部工况，已回退完整曲面力点。')
    panel.add_label('body_info',label='物理参数')
    panel.add_label('torque',label='外力矩（世界 XYZ）')
    panel.add_label('residual',label='最大力残差')
    panel.add_label('timing',label='本次建模与求解',value=f'{result.diagnostics["elapsed_s"]*1000:.2f} ms，全部 {len(cases)} 工况（不含接触提取/绘图）')
    panel.add_label('scale',label='图示比例',value='箭头 3 mm/N；蓝锥高 25 mm，仅表示方向，锥半角 atan(μ)。')
    panel.add_label('meaning',label='解释',value=f'内接 {config.friction_sides} 边锥；可行力可能不唯一。选单个零件查看对应锥和反力。紫箭头为额外外力。')
    panel.add_label('layer_status',label='摩擦锥图层')
    for key,label in (('points','求解点'),('cones','摩擦锥'),('forces','力箭头')):
        panel.add_select(key,label=label,options=['显示','隐藏'],
                         on_change=lambda v,k=key:change(k,v=='显示'))
    def restore_layers():
        for key in objects:
            panel.set_value(key,'显示')
            change(key,True)
    panel.add_button('restore',label='恢复受力图层',on_click=restore_layers)
    redraw()
    return panel
