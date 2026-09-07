#!/usr/bin/env python3
"""
sim_testing_node.py

Testing and verification node for simulation.
Provides testing topics consumed by nodes in mira:
  - /sim/ground_truth_pose (PoseStamped)
  - /sim/odom (Odometry)
  - /sim/points_around_auv (PointCloud2)
  - /master/heading (Float32) - derived from ground truth orientation
  - /master/depth (Float32) - derived from ground truth z
  - gate_pose (PoseStamped) - relative pose to the gate (for gate_navigator)
  - dock_pose (PoseStamped) - relative pose to the dock (for dock_controller)
  - ellipsoid_pose (PoseStamped) - relative pose to the bucket (for phase2)
  - /sim/target_point (Point) - relative target point for ApproachWithDepth/AlignXY
"""

import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import PointCloud2, PointField


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to Euler roll, pitch, yaw (radians)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class SimTestingNode(Node):
    def __init__(self):
        super().__init__('sim_testing_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Ground truth pose subscriber (from ros_gz_bridge)
        self.create_subscription(PoseStamped, '/sim/pose', self._pose_callback, 10)

        # Publishers for mira testing topics
        self.heading_pub = self.create_publisher(Float32, '/master/heading', 10)
        self.depth_pub = self.create_publisher(Float32, '/master/depth', 10)

        # Object pose publishers consumed by mira2_path_planning
        self.gate_pose_pub = self.create_publisher(PoseStamped, 'gate_pose', 10)
        self.dock_pose_pub = self.create_publisher(PoseStamped, 'dock_pose', 10)
        self.ellipsoid_pose_pub = self.create_publisher(PoseStamped, 'ellipsoid_pose', 10)

        # Target point publisher for mira2_actions (ApproachWithDepth, AlignXY)
        self.target_point_pub = self.create_publisher(Point, '/sim/target_point', 10)

        # Known object world coordinates (default pool_world / sauvc25 positions)
        self.declare_parameter('gate_world_pos', [-15.0, 0.0, -0.2])
        self.declare_parameter('dock_world_pos', [-1.0, 10.0, -1.8])
        self.declare_parameter('bucket_world_pos', [1.0, 10.0, -2.0])

        # Current AUV state in world frame
        self.auv_pos = [11.0, -11.5, -0.2]
        self.auv_quat = [0.0, 0.0, 0.7071, 0.7071]
        self.auv_rpy = [0.0, 0.0, 1.5708]
        self.has_pose = False

        # Periodic timer for publishing derived testing topics (20 Hz)
        self.timer = self.create_timer(0.05, self._publish_testing_topics)
        self.get_logger().info('SimTestingNode initialized: publishing ground truth and /sim testing topics')

    def _pose_callback(self, msg: PoseStamped):
        self.auv_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.auv_quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        self.auv_rpy = euler_from_quaternion(*self.auv_quat)
        self.has_pose = True

    def _odom_callback(self, msg: Odometry):
        if not self.has_pose:
            self.auv_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            self.auv_quat = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            self.auv_rpy = euler_from_quaternion(*self.auv_quat)
            self.has_pose = True

    def _publish_testing_topics(self):
        now = self.get_clock().now().to_msg()
        yaw = self.auv_rpy[2]

        # 1. /master/heading in degrees (0 to 360, East=0, North=90 or Compass North=0)
        heading_deg = math.degrees(yaw) % 360.0
        heading_msg = Float32()
        heading_msg.data = float(heading_deg)
        self.heading_pub.publish(heading_msg)

        # 2. /master/depth (positive downwards)
        depth_msg = Float32()
        depth_msg.data = float(-self.auv_pos[2])
        self.depth_pub.publish(depth_msg)

        # 3. Relative Object Poses
        gate_w = self.get_parameter('gate_world_pos').value
        dock_w = self.get_parameter('dock_world_pos').value
        bucket_w = self.get_parameter('bucket_world_pos').value

        def make_relative_pose(obj_pos):
            # Transform world offset (dx, dy, dz) to vehicle body frame (forward, left, up)
            dx = obj_pos[0] - self.auv_pos[0]
            dy = obj_pos[1] - self.auv_pos[1]
            dz = obj_pos[2] - self.auv_pos[2]

            # Rotate by -yaw
            cos_y = math.cos(-yaw)
            sin_y = math.sin(-yaw)
            bx = cos_y * dx - sin_y * dy
            by = sin_y * dx + cos_y * dy
            bz = dz

            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'base_link'
            ps.pose.position.x = float(bx)
            ps.pose.position.y = float(by)
            ps.pose.position.z = float(bz)
            ps.pose.orientation.w = 1.0
            return ps, bx, by, bz

        gate_rel, gx, gy, gz = make_relative_pose(gate_w)
        dock_rel, _, _, _ = make_relative_pose(dock_w)
        bucket_rel, _, _, _ = make_relative_pose(bucket_w)

        self.gate_pose_pub.publish(gate_rel)
        self.dock_pose_pub.publish(dock_rel)
        self.ellipsoid_pose_pub.publish(bucket_rel)

        # 4. Target Point
        target_pt = Point(x=gx, y=gy, z=gz)
        self.target_point_pub.publish(target_pt)


def main(args=None):
    rclpy.init(args=args)
    node = SimTestingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
