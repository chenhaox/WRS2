"""Compare measured points with translucent copies of the original meshes.

Run from the repository root: python -m examples.virtual_d405_model_overlay
"""

import numpy as np

from wrs import wss, wssop, wvw
from wrs.sensor import VirtualD405


scene = wss.Scene()
wssop.icosphere(pos=(-.055, -.015, .29), radius=.045, subdivisions=3,
               rgb=(1, .4, .03)).add_to_scene(scene)
wssop.box(pos=(.06, .025, .37), xyz_lengths=(.08, .07, .08),
          rgb=(.15, .55, .9)).add_to_scene(scene)
wssop.cylinder(spos=(-.12, .065, .40), epos=(.12, .065, .40), radius=.008,
               rgb=(.35, .18, .06)).add_to_scene(scene)

# Move the optical frame to show why the overlay needs world-frame points.
# These intrinsics are example values, not a real D405 calibration.
with VirtualD405(width=640, height=480, fx=600, fy=600, mode='ideal',
                 pos=(.025, -.015, 0)) as camera:
    frame = camera.capture(scene)

base = wvw.World(cam_pos=(.4, -.45, .12), cam_lookat_pos=(0, 0, .32))
base.set_caption('VirtualD405 - cyan samples / translucent source meshes')

# Transparency belongs to display copies; the capture scene remains opaque.
for obj in scene:
    ghost = obj.clone()
    ghost.alpha = .22
    ghost.add_to_scene(base.scene)

points, colors = frame.get_point_cloud(world_frame=True, stride=6)
# Cyan distinguishes measurements from the original model colors.
display_colors = np.full_like(colors, (0, .9, 1))
wssop.point_cloud(points, display_colors).add_to_scene(base.scene)
base.run()
