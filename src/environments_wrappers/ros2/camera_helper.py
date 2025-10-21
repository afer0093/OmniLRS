"""
Camera utility module for ROS2 wrapper.

This module provides helper functions to capture camera images from Omniverse
USD prims and convert them to ROS Image messages.

Author: Modified for OmniLRS
"""

import numpy as np
from typing import Optional, Tuple


def get_rgb_image_from_viewport(
    prim_path: str,
    width: int = 512,
    height: int = 512,
) -> Optional[np.ndarray]:
    """
    Captures RGB image from a camera prim path using Omniverse viewport rendering.

    Args:
        prim_path (str): The USD prim path to the camera (e.g., "/Lunalab/Cameras/TopView")
        width (int): Output image width in pixels
        height (int): Output image height in pixels

    Returns:
        Optional[np.ndarray]: RGB image as numpy array (H, W, 3) in range [0, 255]
                              or None if capture fails.
    """
    try:
        from omni.kit.viewport.utility import get_active_viewport_window
        from omni.replicator.core import RendererProductName
        import omni.replicator.core as rep

        # Get the prim from stage
        from pxr import Usd
        stage = Usd.Stage.GetCurrentStage()
        camera_prim = stage.GetPrimAtPath(prim_path)
        
        if not camera_prim.IsValid():
            print(f"Camera prim at {prim_path} is not valid")
            return None

        # Configure the viewport to use this camera
        viewport = get_active_viewport_window()
        if viewport:
            viewport.get_viewport_interface().set_camera_prim_path(prim_path)

        # Capture RGB image
        with rep.trigger.on_frame():
            rgb_data = rep.get(
                RendererProductName.RGB,
                minibatch_size=1,
                width=width,
                height=height,
            )

        if rgb_data is not None and len(rgb_data) > 0:
            # Convert from [0, 1] float to [0, 255] uint8
            image = np.uint8(rgb_data[0] * 255)
            return image

        return None

    except Exception as e:
        print(f"Error capturing image from {prim_path}: {str(e)}")
        return None


def get_depth_image_from_viewport(
    prim_path: str,
    width: int = 512,
    height: int = 512,
) -> Optional[np.ndarray]:
    """
    Captures depth image from a camera prim path.

    Args:
        prim_path (str): The USD prim path to the camera
        width (int): Output image width in pixels
        height (int): Output image height in pixels

    Returns:
        Optional[np.ndarray]: Depth image as numpy array (H, W) or None if capture fails.
    """
    try:
        from omni.kit.viewport.utility import get_active_viewport_window
        from omni.replicator.core import RendererProductName
        import omni.replicator.core as rep

        viewport = get_active_viewport_window()
        if viewport:
            viewport.get_viewport_interface().set_camera_prim_path(prim_path)

        with rep.trigger.on_frame():
            depth_data = rep.get(
                RendererProductName.DISTANCE_TO_CAMERA,
                minibatch_size=1,
                width=width,
                height=height,
            )

        if depth_data is not None and len(depth_data) > 0:
            return depth_data[0]

        return None

    except Exception as e:
        print(f"Error capturing depth from {prim_path}: {str(e)}")
        return None


def get_camera_matrices(prim_path: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    Retrieves camera intrinsic and extrinsic matrices from a USD camera prim.

    Args:
        prim_path (str): The USD prim path to the camera

    Returns:
        Optional[Tuple[np.ndarray, np.ndarray]]: Tuple of (K, P) where:
            - K is the 3x3 camera intrinsic matrix
            - P is the 4x4 camera pose (extrinsic matrix)
            Returns None if camera is not found.
    """
    try:
        from pxr import Usd, UsdGeom, Gf
        import numpy as np

        stage = Usd.Stage.GetCurrentStage()
        camera_prim = stage.GetPrimAtPath(prim_path)

        if not camera_prim.IsValid():
            print(f"Camera prim at {prim_path} is not valid")
            return None

        camera = UsdGeom.Camera(camera_prim)
        if not camera:
            print(f"Prim at {prim_path} is not a camera")
            return None

        # Get camera parameters
        focal_length = camera.GetFocalLengthAttr().Get()
        horizontal_aperture = camera.GetHorizontalApertureAttr().Get()
        vertical_aperture = camera.GetVerticalApertureAttr().Get()

        # Estimate image dimensions (in pixels) - these are typical defaults
        # Adjust as needed based on your rendering setup
        width = 512
        height = 512

        # Compute intrinsic matrix
        fx = (width / horizontal_aperture) * focal_length
        fy = (height / vertical_aperture) * focal_length
        cx = width / 2.0
        cy = height / 2.0

        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)

        # Get camera pose (extrinsic)
        xformable = UsdGeom.Xformable(camera_prim)
        matrix = xformable.GetLocalTransformation()
        P = np.array(matrix, dtype=np.float32)

        return K, P

    except Exception as e:
        print(f"Error getting camera matrices from {prim_path}: {str(e)}")
        return None
