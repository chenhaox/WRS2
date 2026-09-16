"""Display RGB and metric depth from one VirtualD405 capture.

Run from the repository root: python -m examples.virtual_d405_rgbd
This image example requires matplotlib.
"""

import matplotlib.pyplot as plt
import numpy as np

from wrs import wss, wssop
from wrs.sensor import VirtualD405


scene = wss.Scene()
wssop.icosphere(pos=(-.055, -.015, .29), radius=.045, subdivisions=3,
               rgb=(1, .4, .03)).add_to_scene(scene)
wssop.box(pos=(.06, .025, .37), xyz_lengths=(.08, .07, .08),
          rgb=(.15, .55, .9)).add_to_scene(scene)
wssop.cylinder(spos=(-.12, .065, .40), epos=(.12, .065, .40), radius=.008,
               rgb=(.35, .18, .06)).add_to_scene(scene)

# Example intrinsics, not a real D405 calibration. Optical +Z points forward.
with VirtualD405(width=640, height=480, fx=600, fy=600, mode='ideal') as camera:
    frame = camera.capture(scene)

rgb = frame.rgb                # uint8 (H, W, 3), RGB order
depth_m = frame.depth_m         # float32 (H, W), camera Z in meters
depth_raw = frame.depth_raw     # uint16; meters = raw * frame.depth_scale

fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout='constrained')
axes[0].imshow(rgb)
axes[0].set_title('RGB')

# Invalid depth is zero; mask it so it is not drawn as a near surface.
cmap = plt.get_cmap('viridis').with_extremes(bad='black')
depth_image = axes[1].imshow(np.ma.masked_where(~frame.valid_mask, depth_m),
                            cmap=cmap, vmin=camera.min_depth, vmax=camera.max_depth,
                            interpolation='nearest')
axes[1].set_title('Depth (black = invalid)')
fig.colorbar(depth_image, ax=axes[1], label='Camera Z (m)', shrink=.85)
for ax in axes:
    ax.set_xlabel('u (pixels)')
    ax.set_ylabel('v (pixels)')

plt.show()
