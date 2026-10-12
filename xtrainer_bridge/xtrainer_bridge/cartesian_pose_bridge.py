#!/usr/bin/env python3
"""Bridge ROS Pose/PoseStamped messages to and from the Dobot Cartesian API.

The Dobot driver uses ``ServoP(x, y, z, rx, ry, rz)`` with position in mm and
Euler orientation in degrees.  This node exposes a ROS-friendly interface:

* subscribes to either ``geometry_msgs/msg/Pose`` or ``PoseStamped``;
* transforms commands into ``base_frame`` when a TF is available;
* forwards the latest command through the driver's ``ServoP`` service; and
* publishes the driver's ``ToolVectorActual`` feedback as ``PoseStamped``.
"""

import math
from typing import Optional

import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped with tf2)
import tf2_ros
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import ServoP
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node


def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Return an ``(x, y, z, w)`` quaternion for roll/pitch/yaw radians."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _euler_from_quaternion(x: float, y: float, z: float, w: float):
    """Return roll/pitch/yaw radians for an ``(x, y, z, w)`` quaternion."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class CartesianPoseBridge(Node):
    """Expose the Dobot Cartesian API as ROS Pose/PoseStamped topics."""

    def __init__(self):
        super().__init__('cartesian_pose_bridge')

        self.declare_parameter('command_topic', 'cartesian_pose_command')
        self.declare_parameter('command_type', 'PoseStamped')
        self.declare_parameter('command_frame', 'base_link')
        self.declare_parameter('state_topic', 'cartesian_pose_state')
        self.declare_parameter(
            'feedback_topic', 'dobot_msgs_v4/msg/ToolVectorActual')
        self.declare_parameter(
            'servo_p_service', 'dobot_bringup_ros2/srv/ServoP')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('transform_timeout_sec', 0.1)
        self.declare_parameter('dobot_position_unit', 'mm')
        self.declare_parameter('dobot_orientation_unit', 'deg')
        self.declare_parameter('service_retry_period_sec', 0.02)

        self._command_topic = str(self.get_parameter('command_topic').value)
        self._command_type = str(self.get_parameter('command_type').value).lower()
        self._command_frame = str(self.get_parameter('command_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._state_topic = str(self.get_parameter('state_topic').value)
        self._position_unit = str(
            self.get_parameter('dobot_position_unit').value).lower()
        self._orientation_unit = str(
            self.get_parameter('dobot_orientation_unit').value).lower()
        self._transform_timeout = float(
            self.get_parameter('transform_timeout_sec').value)

        if self._command_type not in ('pose', 'posestamped'):
            raise ValueError("command_type must be 'Pose' or 'PoseStamped'")
        if self._position_unit not in ('mm', 'm'):
            raise ValueError("position_unit must be 'mm' or 'm'")
        if self._orientation_unit not in ('deg', 'rad'):
            raise ValueError("orientation_unit must be 'deg' or 'rad'")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._servo_client = self.create_client(
            ServoP, self.get_parameter('servo_p_service').value)
        self._feedback_sub = self.create_subscription(
            ToolVectorActual,
            self.get_parameter('feedback_topic').value,
            self._feedback_callback,
            10,
        )
        self._state_pub = self.create_publisher(PoseStamped, self._state_topic, 10)

        if self._command_type == 'pose':
            self._command_sub = self.create_subscription(
                Pose, self._command_topic, self._pose_callback, 1)
        else:
            self._command_sub = self.create_subscription(
                PoseStamped, self._command_topic, self._pose_stamped_callback, 1)

        self._latest_request: Optional[ServoP.Request] = None
        self._request_in_flight = False
        retry_period = float(
            self.get_parameter('service_retry_period_sec').value)
        self._send_timer = self.create_timer(retry_period, self._send_latest_request)

        self.get_logger().info(
            f'Command: {self._command_topic} ({self._command_type}); '
            f'State: {self._state_topic}; '
            f'ServoP: {self.get_parameter("servo_p_service").value}')

    def _pose_callback(self, msg: Pose):
        stamped = PoseStamped()
        stamped.header.frame_id = self._command_frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = msg
        self._handle_pose(stamped)

    def _pose_stamped_callback(self, msg: PoseStamped):
        stamped = PoseStamped()
        stamped.header = msg.header
        stamped.pose = msg.pose
        if not stamped.header.frame_id:
            stamped.header.frame_id = self._command_frame
        if stamped.header.stamp.sec == 0 and stamped.header.stamp.nanosec == 0:
            stamped.header.stamp = self.get_clock().now().to_msg()
        self._handle_pose(stamped)

    def _handle_pose(self, msg: PoseStamped):
        try:
            msg = self._transform_to_base(msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warning(f'Cannot transform pose to {self._base_frame}: {exc}')
            return

        position_scale = 1000.0 if self._position_unit == 'mm' else 1.0
        roll, pitch, yaw = _euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        if self._orientation_unit == 'deg':
            roll, pitch, yaw = map(math.degrees, (roll, pitch, yaw))

        request = ServoP.Request()
        request.a = msg.pose.position.x * position_scale
        request.b = msg.pose.position.y * position_scale
        request.c = msg.pose.position.z * position_scale
        request.d = roll
        request.e = pitch
        request.f = yaw
        request.param_value = []
        self._latest_request = request
        self._send_latest_request()

    def _transform_to_base(self, msg: PoseStamped) -> PoseStamped:
        if msg.header.frame_id == self._base_frame:
            return msg
        # Importing tf2_geometry_msgs registers PoseStamped so Buffer.transform
        # can apply the translation and quaternion rotation.
        return self._tf_buffer.transform(
            msg,
            self._base_frame,
            timeout=Duration(seconds=self._transform_timeout),
        )

    def _send_latest_request(self):
        if self._request_in_flight or self._latest_request is None:
            return
        if not self._servo_client.service_is_ready():
            return

        request = self._latest_request
        self._latest_request = None
        self._request_in_flight = True
        try:
            future = self._servo_client.call_async(request)
            future.add_done_callback(self._servo_done)
        except Exception as exc:  # Service may disappear between the readiness check and call.
            self._request_in_flight = False
            self._latest_request = request
            self.get_logger().warning(f'Failed to call ServoP: {exc}')

    def _servo_done(self, future):
        try:
            response = future.result()
            if response.res != 0:
                self.get_logger().warning(f'ServoP returned res={response.res}')
        except Exception as exc:
            self.get_logger().warning(f'ServoP call failed: {exc}')
        finally:
            self._request_in_flight = False
            self._send_latest_request()

    def _feedback_callback(self, msg: ToolVectorActual):
        position_scale = 0.001 if self._position_unit == 'mm' else 1.0
        roll, pitch, yaw = msg.rx, msg.ry, msg.rz
        if self._orientation_unit == 'deg':
            roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
        qx, qy, qz, qw = _quaternion_from_euler(roll, pitch, yaw)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._base_frame
        pose.pose.position.x = msg.x * position_scale
        pose.pose.position.y = msg.y * position_scale
        pose.pose.position.z = msg.z * position_scale
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._state_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
