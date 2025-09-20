__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple

from pxr import Gf

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import os

from src.robots.robot import RobotManager


class ROS_RobotManager(Node):
    """
    ROS2 node that manages the robots.
    """

    def __init__(self, RM_conf: dict) -> None:
        super().__init__("Robot_spawn_manager_node")
        self.RM = RobotManager(RM_conf)

        # subscriptions for spawn/teleport/reset
        self.create_subscription(PoseStamped, "/OmniLRS/Robots/Spawn", self.spawn_robot, 1)
        self.create_subscription(PoseStamped, "/OmniLRS/Robots/Teleport", self.teleport_robot, 1)
        self.create_subscription(String, "/OmniLRS/Robots/Reset", self.reset_robot, 1)
        self.create_subscription(Empty, "/OmniLRS/Robots/ResetAll", self.reset_robots, 1)

        # prepare publishers for per-robot link poses (created on demand)
        self._pose_publishers: dict = {}
        # Use TRANSIENT_LOCAL durability so that late subscribers receive the last published message
        self._qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cb_group = ReentrantCallbackGroup()

        # publish timer: will call _publish_all_robot_link_poses at 30Hz
        self._publish_rate_hz = 30.0
        self._timer_period_sec = 1.0 / self._publish_rate_hz
        self._timer = self.create_timer(self._timer_period_sec, self._publish_all_robot_link_poses, callback_group=self._cb_group)

        # TF broadcaster for odom -> robot/base_link and base_link -> link frames
        self._tf_broadcaster = TransformBroadcaster(self)

        self.domain_id = 0
        self.modifications: List[Tuple[callable, dict]] = []

        # Pre-create publishers for configured robots so topics exist even before spawn
        try:
            for rp in getattr(self.RM, "robot_parameters", []):
                robot_name = getattr(rp, "robot_name", None)
                if robot_name:
                    self._ensure_pose_publisher(robot_name)
        except Exception:
            # ignore errors if robot_parameters not iterable or malformed
            pass

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

        # Ensure publishers exist for any robots that were just added
        try:
            for robot_name in list(self.RM.robots.keys()):
                self._ensure_pose_publisher(robot_name)
        except Exception:
            pass

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

    def _ensure_pose_publisher(self, robot_name: str):
        """
        Ensure a PoseArray publisher exists for the given robot_name.
        Topic: /OmniLRS/Robots/<robot_name>/links_pose_array
        """
        if robot_name in self._pose_publishers:
            return self._pose_publishers[robot_name]

        topic = f"/OmniLRS/Robots/{robot_name.strip('/')}/links_pose_array"
        pub = self.create_publisher(PoseArray, topic, qos_profile=self._qos)
        self._pose_publishers[robot_name] = pub
        return pub

    def _publish_all_robot_link_poses(self) -> None:
        """
        Iterate over spawned robots and publish a PoseArray containing each link's world pose.
        Uses RobotRigidGroup.get_pose() when available through RobotManager. This keeps messages
        stamped with current ROS time and robot frame in header.frame_id.
        """
        try:
            for robot_name, robot in self.RM.robots.items():
                # Try to find an associated RobotRigidGroup; fallback to single root pose
                rrg = None
                key_rg = robot_name
                if robot_name in self.RM.robots_RG:
                    rrg = self.RM.robots_RG[robot_name]

                poses_msg = PoseArray()
                poses_msg.header.stamp = self.get_clock().now().to_msg()
                poses_msg.header.frame_id = "odom"

                # broadcast base_link transform under 'odom'
                try:
                    p_root, q_root = robot.get_pose()
                    t_root = TransformStamped()
                    t_root.header.stamp = self.get_clock().now().to_msg()
                    t_root.header.frame_id = "odom"
                    robot_id = robot_name.strip('/')
                    t_root.child_frame_id = f"{robot_id}/base_link"
                    t_root.transform.translation.x = float(p_root[0])
                    t_root.transform.translation.y = float(p_root[1])
                    t_root.transform.translation.z = float(p_root[2])
                    # Robot.get_pose() returns quaternion as (qx,qy,qz,qw)
                    if len(q_root) >= 4:
                        t_root.transform.rotation.x = float(q_root[0])
                        t_root.transform.rotation.y = float(q_root[1])
                        t_root.transform.rotation.z = float(q_root[2])
                        t_root.transform.rotation.w = float(q_root[3])
                    else:
                        t_root.transform.rotation.x = 0.0
                        t_root.transform.rotation.y = 0.0
                        t_root.transform.rotation.z = 0.0
                        t_root.transform.rotation.w = 1.0
                    self._tf_broadcaster.sendTransform(t_root)
                except Exception:
                    # ignore root pose broadcast failure
                    robot_id = robot_name.strip('/')

                if rrg is not None and len(rrg.target_links) > 0:
                    positions, orientations = rrg.get_pose()
                    # positions: (N,3), orientations: (N,4) (w,x,y,z)
                    for p, q in zip(positions, orientations):
                        pose = Pose()
                        pose.position.x = float(p[0])
                        pose.position.y = float(p[1])
                        pose.position.z = float(p[2])
                        # orientation q is assumed (w,x,y,z)
                        pose.orientation.x = float(q[1])
                        pose.orientation.y = float(q[2])
                        pose.orientation.z = float(q[3])
                        pose.orientation.w = float(q[0])
                        poses_msg.poses.append(pose)

                    # broadcast each link transform as child of base_link
                    try:
                        for i, link in enumerate(rrg.target_links):
                            link_name = os.path.basename(link).strip('/') or link.replace('/', '_')
                            t = TransformStamped()
                            t.header.stamp = self.get_clock().now().to_msg()
                            t.header.frame_id = f"{robot_id}/base_link"
                            t.child_frame_id = f"{robot_id}/{link_name}"
                            t.transform.translation.x = float(positions[i][0])
                            t.transform.translation.y = float(positions[i][1])
                            t.transform.translation.z = float(positions[i][2])
                            # orientations from rrg assumed (w,x,y,z)
                            if len(orientations[i]) >= 4:
                                t.transform.rotation.x = float(orientations[i][1])
                                t.transform.rotation.y = float(orientations[i][2])
                                t.transform.rotation.z = float(orientations[i][3])
                                t.transform.rotation.w = float(orientations[i][0])
                            else:
                                t.transform.rotation.x = 0.0
                                t.transform.rotation.y = 0.0
                                t.transform.rotation.z = 0.0
                                t.transform.rotation.w = 1.0
                            self._tf_broadcaster.sendTransform(t)
                    except Exception:
                        pass
                else:
                    # publish single root pose
                    try:
                        p, q = robot.get_pose()
                        pose = Pose()
                        pose.position.x = float(p[0])
                        pose.position.y = float(p[1])
                        pose.position.z = float(p[2])
                        # robot.get_pose returns pose.r as quaternion (x,y,z,w) in some code paths,
                        # but Robot.get_pose docstring says (x,y,z),(qx,qy,qz,qw). Try to adapt.
                        pose.orientation.x = float(q[0]) if len(q) >= 4 else 0.0
                        pose.orientation.y = float(q[1]) if len(q) >= 4 else 0.0
                        pose.orientation.z = float(q[2]) if len(q) >= 4 else 0.0
                        pose.orientation.w = float(q[3]) if len(q) >= 4 else 1.0
                        poses_msg.poses.append(pose)
                    except Exception:
                        # skip if pose retrieval fails
                        continue

                pub = self._ensure_pose_publisher(robot_name)
                pub.publish(poses_msg)
        except Exception as e:
            # Keep node alive on publish errors; log for visibility
            self.get_logger().error(f"Error publishing robot link poses: {e}")

    def cleanRobots(self) -> None:
        """
        Cleans the robots."""

        self.destroy_node()
