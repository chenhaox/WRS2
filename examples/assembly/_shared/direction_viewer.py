"""One small WRS gallery for the nine-case and paper-contact examples."""
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from wrs.assembly import fibonacci_directions, score_assemblability
from .contact_display import contact_layers,draw_contacts


def show_gallery(case_keys, loader, *, initial, port, title, stage=0):
    from wrs import wvw,wssop
    from wrs.viewer.web_ui import Anchor
    base = wvw.World(cam_pos=(.48,-.8,.5),cam_lookat_pos=(0,0,0),port=port)
    pool = ThreadPoolExecutor(max_workers=1)
    settings = dict(case=initial,stage=stage,method='socp',part=None,pending=None,data=None)
    objects, models, dynamic = [], [], []
    panel = base.ui.add_panel('paper',title=title,anchor=Anchor.TOP_RIGHT,width=340,
        movable=True,font_size=12,description='左：当前装配状态的接触；右：世界坐标下的局部拆出方向。')
    panel.add_select('case',label='场景 / 图号',options=list(case_keys),value=initial,
                     on_change=lambda v:settings.update(case=v,stage=0,part=None))
    panel.add_button('load',label='加载所选场景',on_click=lambda:request())
    panel.add_select('method',label='方向算法',options=['socp','fibonacci'],value='socp',
                     on_change=lambda v:change_method(v))
    for key,label in [('title','场景'),('source','数据来源'),('note','模型说明'),('order','检查顺序'),
                      ('result','方向结果'),('score','可装配性'),('timing','时间'),('issues','待解决项')]:
        panel.add_label(key,label=label,value='')
    panel.add_label('legend',label='方向颜色',value='蓝：拆出；紫：反向插入；绿点：Fibonacci 可行方向。SOCP 只画一个最优方向。')
    panel.add_label('scope',label='验证范围',value='局部平移方向，未验证有限路径、抓取、稳定性或机器人执行。unknown 时不画可执行方向。')

    def change_method(value):
        settings['method']=value
        if settings['data'] is not None: draw()

    def change_part(value):
        settings['part']=value; draw()

    def change_stage(value):
        settings['stage']=int(value); settings['part']=None; request(reuse_case=True)

    def draw():
        data = settings['data']
        assembly,state,analysis,results,meta = data
        for obj in [*objects,*models]: base.scene.remove(obj)
        if models: base.ui.remove_panel('contacts')
        objects.clear(); models.clear()
        pid = settings['part']
        result = results[pid][settings['method']]
        # Display-only uniform scale; all computations/reports remain in metres.
        world = {p.part_id:p.geometry.vertices@state.poses[p.part_id][:3,:3].T+state.poses[p.part_id][:3,3]
                 for p in assembly.parts if p.part_id in state.poses}
        vertices = np.vstack(list(world.values()))
        center = (vertices.max(0)+vertices.min(0))/2
        scale = .25/max(np.ptp(vertices,axis=0))
        left,right = np.array([-.16,0,0]),np.array([.16,0,0])
        colors = ((.30,.43,.91),(.88,.25,.23),(.93,.70,.16),(.49,.29,.14),(.67,.27,.86),(.62,.66,.68))
        movable = [p for p in assembly.parts if not p.fixed]
        for p in assembly.parts:
            if p.part_id not in world: continue
            color = (.62,.68,.72) if p.fixed else colors[movable.index(p)%len(colors)]
            model = wssop.mesh((world[p.part_id]-center)*scale+left,p.geometry.faces,
                              rgb=color,name=p.part_id)
            models.append(model); model.add_to_scene(base.scene)
        layers = contact_layers(analysis.patches)
        for layer in layers:
            for key in ('cells', 'boundary_loops'):
                layer[key]=[(np.asarray(c)-center)*scale+left for c in layer[key]]
            layer['points']=(layer['points']-center)*scale+left
        objects.extend(draw_contacts(base,layers,models,description=f'当前选中零件：{pid}。显示当前状态全部接触；尺寸只为显示等比例缩放。'))
        outline = fibonacci_directions(650)
        objects.append(wssop.point_cloud(right+.085*outline,np.tile((.65,.70,.77),(len(outline),1)),alpha=.22))
        # A mathematically feasible cone with unresolved geometry is not certified.
        displayable = result.status in ('feasible','unconstrained')
        if displayable:
            if result.method=='fibonacci' and len(result.directions):
                if len(result.directions)>2:
                    objects.append(wssop.point_cloud(right+.085*result.directions,
                        np.tile((.03,.7,.4),(len(result.directions),1))))
                else:
                    objects.extend(wssop.sphere(pos=right+.085*d,radius=.004,rgb=(.03,.7,.4)) for d in result.directions)
            if result.best_direction is not None:
                origin = (world[pid].mean(0)-center)*scale+left
                for start in (right,origin):
                    for sign,color in ((1,(.1,.35,.95)),(-1,(.65,.18,.84))):
                        objects.append(wssop.arrow(start,start+sign*.095*result.best_direction,
                            shaft_radius=.0012,head_radius=.0035,head_length=.009,rgb=color))
        # draw_contacts already installs overlays; scene.add is idempotent.
        for obj in objects: base.scene.add(obj)
        quality = score_assemblability(result)
        panel.set_value('result',f'{pid}: {result.status}；维数 {result.dimension}；显示方向 {len(result.directions) if displayable else 0}')
        panel.set_value('score',f'类别 {quality.case}；A={quality.score if quality.score is not None else "未求解"}')
        panel.set_value('issues','；'.join(map(str,result.issues)) or '无局部接触问题')
        panel.set_value('timing',f'接触 {1000*meta.get("contact_s",0):.1f} ms；'
            f'SOCP {1000*results[pid]["socp"].diagnostics.get("elapsed_s",0):.2f} ms；'
            f'Fibonacci {1000*results[pid]["fibonacci"].diagnostics.get("elapsed_s",0):.2f} ms')

    def install(data):
        settings['data']=data
        assembly,state,analysis,results,meta = data
        settings['loaded_case']=settings['case']
        for key in dynamic: panel.remove(key)
        dynamic.clear()
        order = tuple(meta['order'])
        settings['part'] = settings['part'] if settings['part'] in results else list(results)[-1]
        panel.add_select('part',label='计算哪个零件的方向',options=list(results),value=settings['part'],on_change=change_part)
        dynamic.append('part')
        if len(order)>1:
            panel.add_select('stage',label='中间状态：0 完整，1..N 前缀',options=[str(i) for i in range(len(order)+1)],
                             value=str(settings['stage']),on_change=change_stage)
            dynamic.append('stage')
        for key in ('title','source','note'): panel.set_value(key,meta.get(key,''))
        panel.set_value('order',' → '.join(order))
        draw()

    def request(reuse_case=False):
        if settings['pending'] is not None: return
        if reuse_case: settings['case']=settings['loaded_case']; panel.set_value('case',settings['case'])
        panel.set_value('result','正在计算接触与方向……')
        for key in ('case','load',*dynamic): panel.set_enabled(key,False)
        settings['pending']=pool.submit(loader,settings['case'],settings['stage'])

    def tick(dt):
        future=settings['pending']
        if future is not None and future.done():
            settings['pending']=None
            try: install(future.result())
            except Exception as exc: panel.set_value('result',f'加载失败：{exc}')
            for key in ('case','load',*dynamic): panel.set_enabled(key,True)

    request(); base.schedule_interval(tick,.1)
    try: base.run()
    finally: pool.shutdown(wait=False,cancel_futures=True)
