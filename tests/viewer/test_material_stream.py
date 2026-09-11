"""Regression: colour/opacity edits must reach connected and replayed viewers."""
import asyncio
import unittest
import numpy as np
from wrs import wssop
from wrs.viewer.world import World
from wrs.viewer import protocol
from wrs.viewer.server import Hub


class MaterialStreamTests(unittest.TestCase):
    def test_rgba_updates_reuse_geometry_and_unchanged_frames_send_nothing(self):
        async def run():
            world = World(auto_start_hub=False,hz=10000)
            obj = wssop.box(alpha=.2)
            world.scene.add(obj)
            snap = list(protocol.iter_scene_models(world.scene))
            known = set()
            models,geoms = protocol.describe([(i,m) for i,m,_ in snap],known)
            live = {i:protocol.model_stamp(m) for i,m,_ in snap}
            frames = []
            class Socket:
                async def send(self,message):
                    frames.append(message)
            ws = Socket()
            task = asyncio.create_task(world._send_scene(ws,live,known))
            try:
                await asyncio.sleep(.01)
                frames.clear()
                await asyncio.sleep(.01)
                self.assertEqual(frames,[])
                for alpha in (0.,1.,.4):
                    obj.alpha = alpha
                    await asyncio.sleep(.01)
                    headers = [protocol.unpack(f)[0] for f in frames if isinstance(f,bytes)]
                    deltas = [h for h in headers if h['type']=='scene_delta']
                    self.assertEqual(len(deltas),1)
                    self.assertEqual(deltas[0]['geometries'],[])
                    self.assertEqual(deltas[0]['remove'],[])
                    self.assertEqual(deltas[0]['models'][0]['rgba'][3],alpha)
                    self.assertEqual(deltas[0]['models'][0]['id'],models[0]['id'])
                    frames.clear()
                obj.rgb = (.9,.1,.3)
                await asyncio.sleep(.01)
                header = next(protocol.unpack(f)[0] for f in frames if isinstance(f,bytes))
                np.testing.assert_allclose(header['models'][0]['rgba'],[.9,.1,.3,.4],atol=1e-7)
            finally:
                world._closed = True
                await task
        asyncio.run(run())

    def test_hub_replays_updated_material_without_new_geometry(self):
        async def run():
            world = World(auto_start_hub=False)
            obj = wssop.box(alpha=.2)
            world.scene.add(obj)
            snap = list(protocol.iter_scene_models(world.scene))
            mid,model,_ = snap[0]
            known = set()
            entries,geometries = protocol.describe([(mid,model)],known)
            hub = Hub(auto_open=False)
            await hub._relay(protocol.scene_message('scene_init',entries,geometries))
            obj.alpha = 1.
            entries,geometries = protocol.describe([(mid,model)],known)
            self.assertFalse(geometries)
            await hub._relay(protocol.scene_message('scene_delta',entries,geometries,remove=[]))
            class Viewer:
                def __init__(self): self.frames = []
                async def send(self,message): self.frames.append(message)
                def __aiter__(self): return self
                async def __anext__(self): raise StopAsyncIteration
            viewer = Viewer()
            await hub.on_view(viewer)
            replay = protocol.unpack(viewer.frames[0])[0]
            self.assertTrue(replay['replay'])
            self.assertEqual(replay['models'][0]['rgba'][3],1.)
            self.assertEqual(len(replay['geometries']),1)
        asyncio.run(run())


if __name__=='__main__': unittest.main()
