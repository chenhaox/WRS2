"""D405 nominal preset for the common WRS virtual depth pipeline."""

from .camera_model import CameraModel
from .virtual_depth_camera import VirtualDepthCamera


class VirtualD405(VirtualDepthCamera):
    """Passive-stereo D405 approximation, anchored at the left optical frame.

    Defaults: 640x480, nominal 87x58 degree FOV, 18 mm baseline, 7--50 cm
    operating range. FOV-derived intrinsics are approximate and mode dependent;
    supply camera_model from the active real device profile for calibration.
    The 0.1 mm depth_scale is a selectable simulation encoding, not a claim
    about firmware defaults or measurement accuracy. Random noise is off by
    default; d405_fast geometric occlusion is on. Set mode='ideal' for truth.

    RGB and depth share the left optical view. No projector, separate RGB
    camera, or D4 firmware matcher is simulated. Other keywords are forwarded
    to VirtualDepthCamera (intrinsics, distortion, pose, noise, seed, near/far,
    fps, T_mount_camera, mode, name).
    """

    def __init__(self, camera_model=None, *, width=None, height=None,
                 min_depth=0.07, max_depth=0.5, depth_scale=0.0001,
                 baseline_m=0.018, **kwargs):
        if camera_model is None and 'fx' not in kwargs and 'fy' not in kwargs:
            camera_model = CameraModel.from_fov(640 if width is None else width,
                                               480 if height is None else height, 87.0, 58.0)
            width = height = None
        kwargs.setdefault('name', 'virtual_d405')
        kwargs.setdefault('mode', 'd405_fast')
        kwargs.setdefault('far', max_depth)
        super().__init__(camera_model, width=width, height=height,
                         min_depth=min_depth, max_depth=max_depth,
                         depth_scale=depth_scale, baseline_m=baseline_m, **kwargs)
