__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Dict, List, Tuple

from pxr import Gf

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from src.robots.robot import RobotManager


class ROS_RobotManager(Node):
    """
    ROS2 node that manages the robots.
    """

    def __init__(self, RM_conf: dict) -> None:
        super().__init__("Robot_spawn_manager_node")
        self.RM = RobotManager(RM_conf)

        self.create_subscription(PoseStamped, "/OmniLRS/Robots/Spawn", self.spawn_robot, 1)
        self.create_subscription(PoseStamped, "/OmniLRS/Robots/Teleport", self.teleport_robot, 1)
        self.create_subscription(String, "/OmniLRS/Robots/Reset", self.reset_robot, 1)
        self.create_subscription(Empty, "/OmniLRS/Robots/ResetAll", self.reset_robots, 1)

        self.domain_id = 0
        self.modifications: List[Tuple[callable, dict]] = []
        # Lazily created publishers for each robot's base_link pose
        self._base_link_pose_pubs: Dict[str, any] = {}

    def _get_pose_pub(self, robot_name: str):
        """
        Get or create a PoseStamped publisher for the robot's base_link.

        Topic: /<robot_name>/base_link/pose (robot_name starts with "/")
        """
        # normalize robot name to start with '/'
        if robot_name and robot_name[0] != "/":
            robot_name = "/" + robot_name
        if robot_name not in self._base_link_pose_pubs:
            topic = f"{robot_name}/base_link/pose"
            self._base_link_pose_pubs[robot_name] = self.create_publisher(PoseStamped, topic, 10)
        return self._base_link_pose_pubs[robot_name]

    def publish_base_link_poses(self) -> None:
        """
        Publish PoseStamped for each registered robot's base_link in world frame.
        This should be called from the simulation thread after a world step is complete.
        """
        if not self.RM.robots:
            return
        now = self.get_clock().now().to_msg()
        for robot_name, robot in list(self.RM.robots.items()):
            try:
                p, q = robot.get_link_world_pose("base_link")
            except Exception as e:
                # If link is missing or stage not ready, skip publishing for this robot
                self.get_logger().debug(f"Skipping pose publish for {robot_name}: {e}")
                continue
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = "world"
            msg.pose.position.x = float(p[0])
            msg.pose.position.y = float(p[1])
            msg.pose.position.z = float(p[2])
            msg.pose.orientation.x = float(q[0])
            msg.pose.orientation.y = float(q[1])
            msg.pose.orientation.z = float(q[2])
            msg.pose.orientation.w = float(q[3])
            pub = self._get_pose_pub(robot_name)
            pub.publish(msg)

    def reset(self) -> None:
        """
        Resets the robots to their initial state.
        """

        self.clear_modifications()
        self.reset_robots(Empty())

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab.
        """

        self.modifications: List[Tuple[callable, dict]] = []

    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab.
        """

        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()

    def spawn_robot(self, data: PoseStamped) -> None:
        """
        Spawns a robot.

        Args:
            data (String): Name and path of the robot to spawn.
                           Must be in the format: robot_name:usd_path
        """

        assert len(data.header.frame_id.split(":")) == 2, "The data should be in the format: robot_name:usd_path"
        robot_name, usd_path = data.header.frame_id.split(":")
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
        self.modifications.append(
            [
                self.RM.add_robot,
                {"usd_path": usd_path, "robot_name": robot_name, "p": p, "q": q, "domain_id": self.domain_id},
            ]
        )

    def teleport_robot(self, data: PoseStamped) -> None:
        """
        Teleports a robot.

        Args:
            data (Pose): Pose in ROS2 Pose format.
        """

        robot_name = data.header.frame_id
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.modifications.append([self.RM.teleport_robot, {"robot_name": robot_name, "position": p, "orientation": q}])

    def reset_robot(self, data: String) -> None:
        """
        Resets a robot.

        Args:
            data (String): Name of the robot to reset.
        """

        robot_name = data.data
        self.modifications.append([self.RM.reset_robot, {"robot_name": robot_name}])

    def reset_robots(self, data: Empty) -> None:
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM.reset_robots, {}])

    def cleanRobots(self) -> None:
        """
        Cleans the robots."""

        self.destroy_node()
