"""Contact overlays and two small controls, shared by the WRS examples."""
import numpy as np


def contact_layers(patches):
    """Keep actual region cells, including holes; never fill their convex hull."""
    return [dict(name=f'{p.part_a} / {p.part_b}', kind=p.classification,
                 dimension=p.dimension, cells=[c for r in p.regions for c in r.cells_world_m],
                 points=p.points_a_world_m) for p in patches]


def _boundary_segments(cells):
    # Count shared cell edges: internal triangulation edges cancel.
    edges = {}
    for cell in cells:
        if len(cell) < 3:
            continue
        for a,b in zip(cell,np.roll(cell,-1,axis=0)):
            key = tuple(sorted((tuple(a),tuple(b))))
            count,_ = edges.get(key,(0,(a,b)))
            edges[key] = count+1,(a,b)
    return [segment for count,segment in edges.values() if count == 1]


def draw_contacts(base, layers, part_models, *, offset=(0,0,0), description=''):
    """Draw evidence only. UI changes visibility/opacity, never physical state."""
    from wrs import wssop
    from wrs.assembly.visualization import COLORS
    from wrs.viewer.web_ui import Anchor
    colors = {**COLORS, 'declared':'#329bc9'}
    offset = np.asarray(offset)
    overlays = []
    for layer in layers:
        color = colors[layer['kind']]
        rgb = np.array([int(color[i:i+2],16)/255 for i in (1,3,5)])
        cells = [np.asarray(c)+offset for c in layer['cells']]
        if layer['dimension'] == 2:
            vertices, faces = [], []
            for cell in cells:
                start = len(vertices)
                vertices.extend(cell)
                faces.extend((start,start+i,start+i+1) for i in range(1,len(cell)-1))
            if faces:
                # Separate front/back meshes preserve normals under back-face culling.
                for winding in (np.asarray(faces),np.asarray(faces)[:,::-1]):
                    overlays.append(wssop.mesh(vertices,winding,rgb=rgb,alpha=.88,
                                               name=f"contact: {layer['name']} [{layer['kind']} ]"))
            segments = _boundary_segments(cells)
        elif layer['dimension'] == 1:
            segments = [pair for cell in cells for pair in zip(cell[:-1],cell[1:])]
        else:
            segments = []
            for point in layer['points']:
                overlays.append(wssop.sphere(pos=point+offset,radius=.001,rgb=rgb))
        if segments:
            overlays.append(wssop.linsegs(segments,radius=.0003,srgbs=rgb,alpha=.95))
    for obj in overlays:
        base.scene.add(obj)

    panel = base.ui.add_panel('contacts',title='装配接触面',anchor=Anchor.TOP_LEFT,
                               width=250,offset=16,font_size=12,movable=True,
                               description=description)
    panel.add_label('legend',label='颜色',value='绿 active · 橙 near · 红 interference · 紫 unknown')
    if any(layer['kind']=='declared' for layer in layers):
        panel.add_label('ideal',label='蓝色配合面',value='声明的理想零间隙模型；不是 SDF 检测结果')
    panel.add_label('count',label='区域',value=f'{len(layers)} 块（含点 / 线 / 面）')

    def show_contacts(value):
        for obj in overlays:
            (base.scene.add if value == '显示' else base.scene.remove)(obj)

    def set_opacity(value):
        for model in part_models:
            model.alpha = value

    panel.add_select('show',label='接触面',options=['显示','隐藏'],value='显示',on_change=show_contacts)
    panel.add_slider('opacity',label='零件不透明度',min_value=0,max_value=.8,step=.05,
                      value=.2,on_change=set_opacity)
    set_opacity(.2)
    return overlays
