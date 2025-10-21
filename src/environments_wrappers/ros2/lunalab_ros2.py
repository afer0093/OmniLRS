__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# Custom libs
from src.environments_wrappers.ros2.base_wrapper_ros2 import ROS_BaseManager
from src.environments.lunalab import LunalabController

# Loads ROS2 dependent libraries
from std_msgs.msg import Bool, Float32, ColorRGBA, Int32, Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rclpy
import numpy as np
import cv2
from omni.isaac.core.utils.extensions import enable_extension

# Enable Replicator extension for camera rendering
enable_extension("omni.replicator.core")


class ROS_LunalabManager(ROS_BaseManager):
    """
    ROS2 node that manages the lab environment"""

    def __init__(
        self,
        environment_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab manager.

        Args:
            environment_cfg (dict): Environment configuration.
            **kwargs: Additional arguments.
        """

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        self.LC = LunalabController(**environment_cfg)
        self.LC.load()
        self.trigger_reset = False

        # Initialize camera-related attributes
        self.bridge = CvBridge()
        self.topview_camera_path = "/Lunalab/Cameras/TopView"
        self.image_publisher = self.create_publisher(Image, "/OmniLRS/Lunalab/TopView/image_raw", 1)
        self.camera_info_publisher = self.create_publisher(CameraInfo, "/OmniLRS/Lunalab/TopView/camera_info", 1)
        self.camera_frame_id = "lunalab_topview_camera"
        self.sequence_counter = 0

        self.create_subscription(Bool, "/OmniLRS/Projector/TurnOn", self.set_projector_on, 1)
        self.create_subscription(Float32, "/OmniLRS/Projector/Intensity", self.set_projector_intensity, 1)
        self.create_subscription(Float32, "/OmniLRS/Projector/Radius", self.set_projector_radius, 1)
        self.create_subscription(Pose, "/OmniLRS/Projector/Pose", self.set_projector_pose, 1)
        self.create_subscription(Bool, "/OmniLRS/CeilingLights/TurnOn", self.set_ceiling_on, 1)
        self.create_subscription(Float32, "/OmniLRS/CeilingLights/Intensity", self.set_ceiling_intensity, 1)
        self.create_subscription(Bool, "/OmniLRS/Curtains/Extend", self.set_curtains_mode, 1)
        self.create_subscription(Int32, "/OmniLRS/Terrain/Switch", self.switch_terrain, 1)
        self.create_subscription(Bool, "/OmniLRS/Terrain/EnableRocks", self.enable_rocks, 1)
        self.create_subscription(Int32, "/OmniLRS/Terrain/RandomizeRocks", self.randomize_rocks, 1)

    def periodic_update(self, dt: float) -> None:
        """
        Periodic update to publish camera images.

        Args:
            dt (float): Time step.
        """
        self.publish_topview_camera_image()

    def reset(self) -> None:
        """
        Resets the lab to its initial state."""

        pass

    def set_projector_on(self, data: Bool) -> None:
        """
        Turns the projector on or off.

        Args:
            data (Bool): True to turn the projector on, False to turn it off.
        """

        self.modifications.append([self.LC.turn_projector_on_off, {"flag": data.data}])

    def set_projector_intensity(self, data: Float32) -> None:
        """
        Sets the projector intensity.

        Args:
            data (Float32): Intensity in percentage.
        """

        default_intensity = 300000000.0
        data = default_intensity * float(data.data) / 100.0
        assert data >= 0, "The intensity must be greater than or equal to 0."
        self.modifications.append([self.LC.set_projector_intensity, {"intensity": data}])

    def set_projector_radius(self, data: Float32) -> None:
        """
        Sets the projector radius.

        Args:
            data (Float32): Radius in meters.
        """

        assert data.data > 0.0, "Radius must be greater than 0.0"
        self.modifications.append([self.LC.set_projector_radius, {"radius": data.data}])

    def set_projector_color(self, data: ColorRGBA) -> None:
        """
        Sets the projector color.

        Args:
            data (ColorRGBA): Color in RGBA format.
        """

        color = [data.r, data.g, data.b]
        for c in color:
            assert 0 <= c <= 1, "The color must be between 0 and 1."
        self.modifications.append([self.LC.set_projector_color, {"color": color}])

    def set_projector_pose(self, data: Pose) -> None:
        """
        Sets the projector pose.

        Args:
            data (Pose): Pose in ROS2 Pose format.
        """

        position = (data.position.x, data.position.y, data.position.z)
        orientation = (data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        self.modifications.append([self.LC.set_projector_pose, {"position": position, "orientation": orientation}])

    def set_ceiling_on(self, data: Bool) -> None:
        """
        Turns the ceiling lights on or off.

        Args:
            data (Bool): True to turn the lights on, False to turn them off.
        """

        self.modifications.append([self.LC.turn_room_lights_on_off, {"flag": data.data}])

    def set_ceiling_intensity(self, data: Float32) -> None:
        """
        Sets the ceiling lights intensity.

        Args:
            data (Float32): Intensity in percentage.
        """

        assert data.data >= 0, "The intensity must be greater than or equal to 0."
        self.modifications.append([self.LC.set_room_lights_intensity, {"intensity": data.data}])

    def set_ceiling_radius(self, data: Float32) -> None:
        """
        Sets the ceiling lights radius.

        Args:
            data (Float32): Radius in meters.
        """

        assert data.data > 0.0, "Radius must be greater than 0.0"
        self.modifications.append([self.LC.set_room_lights_radius, {"radius": data.data}])

    def set_ceiling_FOV(self, data: Float32) -> None:
        """
        Sets the ceiling lights field of view.

        Args:
            data (Float32): Field of view in degrees.
        """

        assert 0 <= data.data <= 180, "The field of view must be between 0 and 180."
        self.modifications.append([self.LC.set_room_lights_FOV, {"FOV": data.data}])

    def set_ceiling_color(self, data: ColorRGBA) -> None:
        """
        Sets the ceiling lights color.

        Args:
            data (ColorRGBA): Color in RGBA format.
        """

        color = [data.r, data.g, data.b]
        for c in color:
            assert 0 <= c <= 1, "The color must be between 0 and 1."
        self.modifications.append([self.LC.set_room_lights_color, {"color": color}])

    def set_curtains_mode(self, data: Bool) -> None:
        """
        Sets the curtains mode.

        Args:
            data (Bool): True to extend the curtains, False to retract them.
        """

        self.modifications.append([self.LC.curtains_extend, {"flag": data.data}])

    def switch_terrain(self, data: Int32) -> None:
        """
        Switches the terrain.

        Args:
            data (Int32): 0 for the first terrain, 1 for the second terrain.
        """

        self.modifications.append([self.LC.switch_terrain, {"flag": data.data}])
        self.trigger_reset = True

    def enable_rocks(self, data: Bool) -> None:
        """
        Enables or disables the rocks.

        Args:
            data (Bool): True to enable the rocks, False to disable them.
        """

        self.modifications.append([self.LC.enable_rocks, {"flag": data.data}])
        self.trigger_reset = True

    def randomize_rocks(self, data: Int32) -> None:
        """
        Randomizes the rocks.

        Args:
            data (Int32): Number of rocks to randomize.
        """

        data = int(data.data)
        assert data > 0, "The number of rocks must be greater than 0."
        self.modifications.append([self.LC.randomize_rocks, {"num": data}])
        self.trigger_reset = True

    def get_camera_image_from_prim(self):
        """
        Retrieves the camera image from the specified prim path.
        Uses Omniverse Replicator API to render and capture images.

        Returns:
            np.ndarray: RGB image data from the TopView camera.
        """
        try:
            # Import camera helper
            from src.environments_wrappers.ros2.camera_helper import get_rgb_image_from_viewport

            # Capture RGB image from TopView camera
            image_data = get_rgb_image_from_viewport(
                prim_path=self.topview_camera_path,
                width=512,
                height=512,
            )
            
            return image_data

        except Exception as e:
            self.get_logger().warn(f"Failed to capture camera image: {str(e)}")
            return None

    def publish_topview_camera_image(self) -> None:
        """
        Publishes the TopView camera image as a ROS Image message.
        """
        try:
            # Get image from camera prim
            image_data = self.get_camera_image_from_prim()
            
            if image_data is None:
                return

            # Convert numpy array to ROS Image message
            current_time = self.get_clock().now()
            header = Header()
            header.stamp = current_time.to_msg()
            header.frame_id = self.camera_frame_id
            header.seq = self.sequence_counter
            self.sequence_counter += 1

            # Convert BGR to RGB if needed and create ROS message
            ros_image = self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB),
                encoding="rgb8",
                header=header,
            )

            # Publish image
            self.image_publisher.publish(ros_image)

            # Optionally publish camera info
            self.publish_camera_info(current_time, header)

        except Exception as e:
            self.get_logger().warn(f"Error in publish_topview_camera_image: {str(e)}")

    def publish_camera_info(self, timestamp, header) -> None:
        """
        Publishes the camera calibration info.

        Args:
            timestamp: ROS timestamp
            header: ROS header
        """
        try:
            camera_info = CameraInfo()
            camera_info.header = header

            # Set camera intrinsics (these are default values, adjust as needed)
            camera_info.width = 512
            camera_info.height = 512
            camera_info.distortion_model = "plumb_bob"

            # Camera matrix (fx, fy, cx, cy)
            fx = 512.0  # focal length in pixels
            fy = 512.0
            cx = 256.0  # principal point
            cy = 256.0

            camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
            camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

            camera_info.distortion = [0, 0, 0, 0, 0]

            self.camera_info_publisher.publish(camera_info)

        except Exception as e:
            self.get_logger().warn(f"Error in publish_camera_info: {str(e)}")
