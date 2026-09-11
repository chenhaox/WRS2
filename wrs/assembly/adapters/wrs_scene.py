"""Lazy WRS SceneObject bridge: full visuals and their local transforms."""
import numpy as np
from ..model import MeshData,Part,AssemblyState


def rigid_tf_from_wrs(value):
    """Project float32 rotation roundoff only; reject scale/shear or bad frames."""
    tf=np.asarray(value,dtype=float).copy()
    if (tf.shape!=(4,4) or not np.all(np.isfinite(tf)) or
            not np.allclose(tf[3],[0,0,0,1],rtol=0,atol=1e-7) or
            not np.allclose(tf[:3,:3].T@tf[:3,:3],np.eye(3),rtol=0,atol=2e-6) or np.linalg.det(tf[:3,:3])<=0):
        raise ValueError('Expected WRS rigid transform, without scale/shear')
    u,_,vt=np.linalg.svd(tf[:3,:3]); tf[:3,:3]=u@vt
    return tf


def part_from_scene_object(obj,part_id,*,mass_kg=None,com_local_m=None,friction=None,fixed=False,orientation='auto'):
    vertices,faces=[],[]; offset=0
    for visual in obj.visuals:
        geom=visual.geom
        if geom is None or geom.fs is None or not len(geom.fs): continue
        tf=np.asarray(visual.loc_tf,dtype=float)
        vs=np.asarray(geom.vs,dtype=float)@tf[:3,:3].T+tf[:3,3]
        vertices.append(vs); faces.append(np.asarray(geom.fs,dtype=int)+offset); offset+=len(vs)
    if not vertices: raise ValueError(f'{part_id}: no visual triangle geometry')
    return Part(part_id,MeshData(np.concatenate(vertices),np.concatenate(faces),orientation=orientation),
                rigid_tf_from_wrs(obj.tf),mass_kg=mass_kg,com_local_m=com_local_m,friction=friction,fixed=fixed)


def scene_object_from_part(part,tf=None,*,collision=True,rgb=(.7,.65,.35)):
    from wrs.scene.scene_object import SceneObject
    from wrs.scene.render_model import RenderModel
    from wrs.utils.constant import CollisionType
    obj=SceneObject(name=part.part_id,is_floating=not part.fixed,
                    collision_type=CollisionType.MESH if collision else None)
    obj.add_visual(RenderModel(geom=(part.geometry.vertices.astype(np.float32),part.geometry.faces.astype(np.uint32)),rgb=rgb))
    obj.tf=np.asarray(part.assembled_tf if tf is None else tf,dtype=np.float32)
    return obj


def apply_state_to_scene(state,objects):
    """Update instance poses only, retaining shared immutable geometry."""
    if not isinstance(state,AssemblyState): raise ValueError('AssemblyState required')
    if not set(state.poses).issubset(objects): raise ValueError('Scene is missing present instances')
    for k,tf in state.poses.items(): objects[k].tf=np.asarray(tf,dtype=np.float32)
