#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Wonho Yun

import asyncio
import os
import socket
import threading

from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
import nest_asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vuer import Vuer
from vuer.schemas import Body, MotionControllers

# Allow nested asyncio execution
nest_asyncio.apply()

BODY_HEAD_INDEX = 6  # XRBodyJoint 'head'
BODY_LEFT_ELBOW_INDEX = 10  # XRBodyJoint 'left-arm-lower'
BODY_RIGHT_ELBOW_INDEX = 15  # XRBodyJoint 'right-arm-lower'
# Head-relative VR frame from (head_inverse @ world):
# +Y=forward, +Z=right, +X=down. Convert to ROS (+X forward, +Y left, +Z up).
VR_HEAD_TO_ROS = np.array([
    [0.0, 1.0, 0.0],   # ROS X = head +Y
    [0.0, 0.0, -1.0],  # ROS Y = -head Z
    [-1.0, 0.0, 0.0],  # ROS Z = -head X
], dtype=np.float64)


class VRTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('vr_trajectory_publisher')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.declare_parameter('left_wrist_offset_x', 0.0)
        self.declare_parameter('left_wrist_offset_y', 0.0)
        self.declare_parameter('left_wrist_offset_z', 0.0)
        self.declare_parameter('right_wrist_offset_x', 0.0)
        self.declare_parameter('right_wrist_offset_y', 0.0)
        self.declare_parameter('right_wrist_offset_z', 0.0)
        self.declare_parameter('left_wrist_roll_offset_deg', 90.0)
        self.declare_parameter('left_wrist_pitch_offset_deg', 0.0)
        self.declare_parameter('left_wrist_yaw_offset_deg', 0.0)
        self.declare_parameter('right_wrist_roll_offset_deg', 90.0)
        self.declare_parameter('right_wrist_pitch_offset_deg', 0.0)
        self.declare_parameter('right_wrist_yaw_offset_deg', 0.0)
        self.declare_parameter('left_trigger_offset', 0.0)
        self.declare_parameter('right_trigger_offset', 0.0)
        self.declare_parameter('left_trigger_scale', 1.0)
        self.declare_parameter('right_trigger_scale', 1.0)
        self.declare_parameter('goal_pose_position_scale', 1.1)
        self.declare_parameter('stream_fps', 30)
        self.declare_parameter('pose_publish_hz', 30.0)
        self.declare_parameter('apply_lift_to_arm_z', True)
        self.declare_parameter('lift_to_arm_z_scale', 1.0)
        self.declare_parameter('left_elbow_offset_x', 0.0)
        self.declare_parameter('left_elbow_offset_y', 0.0)
        self.declare_parameter('left_elbow_offset_z', 0.0)
        self.declare_parameter('right_elbow_offset_x', 0.0)
        self.declare_parameter('right_elbow_offset_y', 0.0)
        self.declare_parameter('right_elbow_offset_z', 0.0)

        # VR publishing control flag
        self.vr_publishing_enabled = True  # Default: disabled

        # VR Server setup
        current_dir = os.path.dirname(os.path.abspath(__file__))
        cert_file = os.path.join(current_dir, 'cert.pem')
        key_file = os.path.join(current_dir, 'key.pem')
        hostname = socket.gethostbyname(socket.gethostname())
        ws_url = f'ws://{hostname}:8012'

        self.vuer = Vuer(
            host='0.0.0.0',
            port=8012,
            cert=cert_file,
            key=key_file,
            ws=ws_url,
            queries={'grid': False, 'reconnect': True},
            queue_len=3
        )

        self.fps = int(self.get_parameter('stream_fps').value)
        self.get_logger().info(f'VR Trajectory server available at: https://{hostname}:8012')

        # VR event handlers
        self.vuer.add_handler('BODY_MOVE')(self.on_body_tracking_move)
        self.vuer.add_handler('CONTROLLER_MOVE')(self.on_controller_move)

        # QoS setting
        self.vr_stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.head_joint_pub = self.create_publisher(
            JointTrajectory,
            '/leader/joystick_controller_left/joint_trajectory',
            self.vr_stream_qos
        )

        self.lift_joint_pub = self.create_publisher(
            JointTrajectory,
            '/leader/joystick_controller_right/joint_trajectory',
            self.vr_stream_qos
        )

        self.left_squeeze_pub = self.create_publisher(
            Float32, '/vr_controller/left_squeeze', self.vr_stream_qos
        )
        self.right_squeeze_pub = self.create_publisher(
            Float32, '/vr_controller/right_squeeze', self.vr_stream_qos
        )
        self.left_trigger_pub = self.create_publisher(
            Float32, '/vr_controller/left_trigger', self.vr_stream_qos
        )
        self.right_trigger_pub = self.create_publisher(
            Float32, '/vr_controller/right_trigger', self.vr_stream_qos
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', self.vr_stream_qos
        )

        # Wrist/elbow pose publishers for visualization
        self.left_wrist_rviz_pub = self.create_publisher(
            PoseStamped, '/l_goal_pose', self.vr_stream_qos
        )
        self.right_wrist_rviz_pub = self.create_publisher(
            PoseStamped, '/r_goal_pose', self.vr_stream_qos
        )
        self.left_elbow_rviz_pub = self.create_publisher(
            PoseStamped, '/l_elbow_pose', self.vr_stream_qos
        )
        self.right_elbow_rviz_pub = self.create_publisher(
            PoseStamped, '/r_elbow_pose', self.vr_stream_qos
        )

        # Reactivate service client (call when both A buttons are pressed)
        self.declare_parameter('reactivate_service', '/reactivate')
        self.reactivate_service = str(self.get_parameter('reactivate_service').value)
        self.reactivate_client = self.create_client(Trigger, self.reactivate_service)
        self.both_a_buttons_pressed_prev = False
        self.reactivate_call_in_flight = False
        self.last_reactivate_service_warn_sec = 0.0

        # Subscriber for VR control toggle
        self.vr_control_sub = self.create_subscription(
            Bool,
            '/vr_control/toggle',
            self.vr_control_callback,
            self.vr_stream_qos
        )
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            self.vr_stream_qos
        )

        # VR data storage
        self.left_controller_matrix = None
        self.right_controller_matrix = None
        self.left_controller_state = {}
        self.right_controller_state = {}
        self.left_squeeze_value = 0.0
        self.right_squeeze_value = 0.0
        self.goal_pose_squeeze_threshold = 0.8
        self.head_transform_matrix = np.eye(4)
        self.head_inverse_matrix = np.eye(4)
        self.vr_head_to_ros_rot = R.from_matrix(VR_HEAD_TO_ROS)
        self.camera_to_base_offset = np.array([
            0.0 - 0.0238122 - 0.040 - 0.049483 - 0.0055,  # x: -0.1187952
            0.0 + 0.0 + 0.0 + 0.0 + 0.0,                  # y: 0.0
            -0.01325 + 0.0242094 - 0.054 - 0.102130 - 1.4316  # z: -1.5767706
        ], dtype=np.float64)
        self.wrist_position_offsets = {
            'left': np.array([
                self.get_parameter(
                    'left_wrist_offset_x'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'left_wrist_offset_y'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'left_wrist_offset_z'
                ).get_parameter_value().double_value,
            ], dtype=np.float64),
            'right': np.array([
                self.get_parameter(
                    'right_wrist_offset_x'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'right_wrist_offset_y'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'right_wrist_offset_z'
                ).get_parameter_value().double_value,
            ], dtype=np.float64),
        }
        self.elbow_position_offsets = {
            'left': np.array([
                self.get_parameter(
                    'left_elbow_offset_x'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'left_elbow_offset_y'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'left_elbow_offset_z'
                ).get_parameter_value().double_value,
            ], dtype=np.float64),
            'right': np.array([
                self.get_parameter(
                    'right_elbow_offset_x'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'right_elbow_offset_y'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'right_elbow_offset_z'
                ).get_parameter_value().double_value,
            ], dtype=np.float64),
        }
        self.wrist_rotation_offsets = {
            'left': R.from_euler('xyz', [
                self.get_parameter(
                    'left_wrist_roll_offset_deg'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'left_wrist_pitch_offset_deg'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'left_wrist_yaw_offset_deg'
                ).get_parameter_value().double_value,
            ], degrees=True),
            'right': R.from_euler('xyz', [
                self.get_parameter(
                    'right_wrist_roll_offset_deg'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'right_wrist_pitch_offset_deg'
                ).get_parameter_value().double_value,
                self.get_parameter(
                    'right_wrist_yaw_offset_deg'
                ).get_parameter_value().double_value,
            ], degrees=True),
        }
        self.trigger_offsets = {
            'left': float(self.get_parameter('left_trigger_offset').value),
            'right': float(self.get_parameter('right_trigger_offset').value),
        }
        self.trigger_scales = {
            'left': float(self.get_parameter('left_trigger_scale').value),
            'right': float(self.get_parameter('right_trigger_scale').value),
        }
        self.goal_pose_position_scale = float(self.get_parameter('goal_pose_position_scale').value)
        if not np.isfinite(self.goal_pose_position_scale) or self.goal_pose_position_scale <= 0.0:
            self.get_logger().warn(
                f'Invalid goal_pose_position_scale='
                f'{self.goal_pose_position_scale}; fallback to 1.0'
            )
            self.goal_pose_position_scale = 1.0
        self.pose_publish_hz = float(self.get_parameter('pose_publish_hz').value)
        self.pose_min_period = (1.0 / self.pose_publish_hz) if self.pose_publish_hz > 0.0 else 0.0
        self.last_pose_publish_sec = {
            'left_wrist': 0.0,
            'right_wrist': 0.0,
            'left_elbow': 0.0,
            'right_elbow': 0.0,
        }

        # Thumbstick mode:
        # True: lift + head joints, False: lift + cmd_vel
        self.joystick_mode = True
        self.prev_left_thumbstick_pressed = False
        self.prev_right_thumbstick_pressed = False
        self.linear_x_scale = 3.0
        self.linear_y_scale = 3.0
        self.angular_z_scale = 2.0
        # Match joystick_controller parameters
        self.left_jog_scale = 0.06
        self.right_jog_scale = 0.01
        self.deadzone = 0.05
        # Match sensorxel_l_joy_reverse_interfaces (X/Y reversed) in controller config
        self.left_reverse_x = False
        self.left_reverse_y = True
        self.left_stick_swap_xy = True
        self.right_stick_swap_xy = True
        self.current_joint_states = None
        self.lift_joint_current_position = 0.0
        self.lift_reference_position_for_pose = None
        self.head_joint1_current_position = 0.0
        self.head_joint2_current_position = 0.0
        self.apply_lift_to_arm_z = bool(self.get_parameter('apply_lift_to_arm_z').value)
        self.lift_to_arm_z_scale = float(self.get_parameter('lift_to_arm_z_scale').value)
        self.control_max_hz = 30.0
        self.control_min_period = 1.0 / self.control_max_hz
        self.last_lift_publish_sec = 0.0
        self.last_head_publish_sec = 0.0
        self.last_cmd_vel_publish_sec = 0.0
        self.last_lift_command = None
        self.last_head_command = None
        self.last_cmd_vel_command = (0.0, 0.0, 0.0)

        # Low-pass filter settings
        self.low_pass_filter_alpha = 0.5

        # Logging counters
        self.hand_log_counter = 0
        self.controller_log_counter = 0
        self.log_every_n = self.fps

        # Async setup
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.start_vuer_server()

        self.get_logger().info('VR Trajectory Publisher node has been started')
        self.get_logger().info(
            'VR publishing is DISABLED by default. '
            'Send /vr_control/toggle message (True=enable, False=disable).'
        )
        self.get_logger().info(
            f'Stick swap config: left_stick_swap_xy={self.left_stick_swap_xy}, '
            f'right_stick_swap_xy={self.right_stick_swap_xy}'
        )
        self.get_logger().info(
            f'Wrist offsets | left_pos={self.wrist_position_offsets["left"].tolist()}, '
            f'right_pos={self.wrist_position_offsets["right"].tolist()}'
        )
        self.get_logger().info(
            f'Elbow offsets | left_pos={self.elbow_position_offsets["left"].tolist()}, '
            f'right_pos={self.elbow_position_offsets["right"].tolist()}'
        )
        left_wrist_offsets = [
            self.get_parameter('left_wrist_roll_offset_deg').value,
            self.get_parameter('left_wrist_pitch_offset_deg').value,
            self.get_parameter('left_wrist_yaw_offset_deg').value
        ]
        right_wrist_offsets = [
            self.get_parameter('right_wrist_roll_offset_deg').value,
            self.get_parameter('right_wrist_pitch_offset_deg').value,
            self.get_parameter('right_wrist_yaw_offset_deg').value
        ]
        self.get_logger().info(
            f'Wrist rot offsets deg | left={left_wrist_offsets}, '
            f'right={right_wrist_offsets}'
        )
        self.get_logger().info(
            f'Trigger calibration | '
            f'left=(offset={self.trigger_offsets["left"]:+.3f}, '
            f'scale={self.trigger_scales["left"]:.3f}), '
            f'right=(offset={self.trigger_offsets["right"]:+.3f}, '
            f'scale={self.trigger_scales["right"]:.3f})'
        )
        self.get_logger().info(f'Goal pose position scale={self.goal_pose_position_scale:.3f}')
        self.get_logger().info(
            f'Stream fps={self.fps}, pose publish hz={self.pose_publish_hz:.1f}, '
            f'queue_depth=1'
        )
        self.get_logger().info(
            f'Lift->arm Z coupling: enabled={self.apply_lift_to_arm_z}, '
            f'scale={self.lift_to_arm_z_scale:.3f}'
        )

    def vr_control_callback(self, msg):
        """Enable/disable VR publishing based on message content."""
        new_state = bool(msg.data)  # Read message content

        # Only log if state actually changed
        if new_state != self.vr_publishing_enabled:
            self.vr_publishing_enabled = new_state
            status = 'ENABLED' if self.vr_publishing_enabled else 'DISABLED'
            self.get_logger().info(
                f'VR publishing changed to: {status} (message value: {msg.data})'
            )

            if not self.vr_publishing_enabled:
                self.get_logger().info('VR publishing disabled')

    def is_valid_float(self, value):
        """Check if value is valid float (excluding NaN, inf)."""
        return isinstance(value, (int, float)) and np.isfinite(value)

    def _call_reactivate(self):
        """Call reactivate service without blocking event callbacks."""
        if self.reactivate_call_in_flight:
            return

        if not self.reactivate_client.service_is_ready():
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (now_sec - self.last_reactivate_service_warn_sec) >= 5.0:
                self.get_logger().warn(
                    f'Reactivate service "{self.reactivate_service}" not available'
                )
                self.last_reactivate_service_warn_sec = now_sec
            return

        self.reactivate_call_in_flight = True
        req = Trigger.Request()
        self.reactivate_client.call_async(req).add_done_callback(self._reactivate_done_callback)

    def _reactivate_done_callback(self, future):
        self.reactivate_call_in_flight = False
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Reactivate service called successfully (both A buttons)')
            else:
                self.get_logger().warn(f'Reactivate service returned: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Reactivate service call failed: {e}')

    def apply_deadzone(self, value):
        """Apply deadzone to thumbstick value."""
        abs_value = abs(value)
        if abs_value < self.deadzone:
            return 0.0
        sign = 1.0 if value >= 0.0 else -1.0
        normalized_value = (abs_value - self.deadzone) / (1.0 - self.deadzone)
        return sign * normalized_value

    def calibrate_trigger(self, side, raw_value):
        """Apply trigger offset/scale calibration and clamp to [0, 1]."""
        side_key = side if side in ('left', 'right') else 'left'
        calibrated = (
            (float(raw_value) + self.trigger_offsets[side_key])
            * self.trigger_scales[side_key]
        )
        return float(np.clip(calibrated, 0.0, 1.0))

    def joint_states_callback(self, msg):
        """Receive current joint states for incremental joystick control."""
        self.current_joint_states = msg
        if 'lift_joint' in msg.name:
            idx = msg.name.index('lift_joint')
            self.lift_joint_current_position = msg.position[idx]
            if self.lift_reference_position_for_pose is None:
                self.lift_reference_position_for_pose = self.lift_joint_current_position
        if 'head_joint1' in msg.name:
            idx = msg.name.index('head_joint1')
            self.head_joint1_current_position = msg.position[idx]
        if 'head_joint2' in msg.name:
            idx = msg.name.index('head_joint2')
            self.head_joint2_current_position = msg.position[idx]

    def get_lift_z_delta_for_arm_pose(self):
        """Return Z delta applied to arm goals from lift joint motion."""
        if not self.apply_lift_to_arm_z:
            return 0.0
        if self.lift_reference_position_for_pose is None:
            return 0.0
        return (
            (self.lift_joint_current_position - self.lift_reference_position_for_pose)
            * self.lift_to_arm_z_scale
        )

    def safe_point(self, x, y, z):
        """Create safe Point (filtering NaN/inf values)."""
        safe_x = float(x) if self.is_valid_float(x) else 0.0
        safe_y = float(y) if self.is_valid_float(y) else 0.0
        safe_z = float(z) if self.is_valid_float(z) else 0.0
        return Point(x=safe_x, y=safe_y, z=safe_z)

    def safe_quaternion(self, x, y, z, w):
        """Create safe Quaternion (filtering NaN/inf values)."""
        safe_x = float(x) if self.is_valid_float(x) else 0.0
        safe_y = float(y) if self.is_valid_float(y) else 0.0
        safe_z = float(z) if self.is_valid_float(z) else 0.0
        safe_w = float(w) if self.is_valid_float(w) else 1.0
        return Quaternion(x=safe_x, y=safe_y, z=safe_z, w=safe_w)

    def matrix_to_pose(self, mat):
        """Convert 4x4 transformation matrix to (position, quaternion)."""
        pos = mat[:3, 3]
        rot = mat[:3, :3]

        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(rot)):
            self.get_logger().warn('Invalid matrix data detected, using default pose')
            return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])

        trace = rot[0, 0] + rot[1, 1] + rot[2, 2]

        if 1 + trace <= 0:
            quat = np.array([0.0, 0.0, 0.0, 1.0])
            return pos, quat

        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (rot[2, 1] - rot[1, 2]) / s
            qy = (rot[0, 2] - rot[2, 0]) / s
            qz = (rot[1, 0] - rot[0, 1]) / s
        elif ((rot[0, 0] > rot[1, 1]) and (rot[0, 0] > rot[2, 2])):
            s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif rot[1, 1] > rot[2, 2]:
            s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s

        quat = np.array([qx, qy, qz, qw])

        if not np.all(np.isfinite(quat)):
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            norm = np.linalg.norm(quat)
            if norm > 0:
                quat = quat / norm
            else:
                quat = np.array([0.0, 0.0, 0.0, 1.0])

        return pos, quat

    def vr_to_ros_transform(self, vr_pos, vr_quat):
        """Convert head-relative VR pose to ROS pose (no hand-specific offsets)."""
        ros_pos = (VR_HEAD_TO_ROS @ vr_pos).astype(np.float64)
        vr_rotation = R.from_quat([vr_quat[0], vr_quat[1], vr_quat[2], vr_quat[3]])
        ros_rotation = self.vr_head_to_ros_rot * vr_rotation * self.vr_head_to_ros_rot.inv()
        return ros_pos, ros_rotation.as_quat()

    def can_publish_goal_pose(self):
        """Safety gate for goal_pose topics."""
        return (
            self.vr_publishing_enabled and
            self.left_squeeze_value >= self.goal_pose_squeeze_threshold and
            self.right_squeeze_value >= self.goal_pose_squeeze_threshold
        )

    def apply_wrist_offsets(self, side, position_ros, rotation_ros):
        """Apply per-hand offsets after VR->ROS conversion."""
        side_key = side if side in ('left', 'right') else 'left'
        position_with_offset = position_ros + self.wrist_position_offsets[side_key]
        # Local wrist frame rotation offset.
        rotation_with_offset = rotation_ros * self.wrist_rotation_offsets[side_key]
        return position_with_offset, rotation_with_offset

    def scale_goal_position(self, position_ros):
        """Scale head-relative arm target position before base/camera offsets."""
        return np.asarray(position_ros, dtype=np.float64) * self.goal_pose_position_scale

    def get_body_joint_matrix_from_flat(self, body_array, joint_index):
        """Extract a 4x4 body joint matrix from flattened BODY_MOVE array."""
        start_idx = joint_index * 16
        end_idx = start_idx + 16
        if body_array.size < end_idx:
            return None
        joint_matrix = np.asarray(
            body_array[start_idx:end_idx], dtype=np.float64
        ).reshape(4, 4, order='F')
        if not np.all(np.isfinite(joint_matrix)):
            return None
        if abs(float(np.linalg.det(joint_matrix[:3, :3]))) < 1e-6:
            return None
        return joint_matrix

    def _publish_wrist_pose_from_matrix(self, world_joint_matrix, side):
        """Publish wrist pose from a world transform matrix."""
        try:
            if not self.can_publish_goal_pose():
                return
            pose_key = f'{side}_wrist'
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (self.pose_min_period > 0.0 and
                    (now_sec - self.last_pose_publish_sec[pose_key])
                    < self.pose_min_period):
                return

            relative_joint_matrix = self.head_inverse_matrix @ world_joint_matrix
            relative_pos_head, relative_quat_head = self.matrix_to_pose(
                relative_joint_matrix
            )
            relative_pos_ros, relative_quat_ros = self.vr_to_ros_transform(
                relative_pos_head, relative_quat_head
            )
            relative_pos_ros = self.scale_goal_position(relative_pos_ros)
            relative_rot_ros = R.from_quat(relative_quat_ros)

            # 1) Coordinate conversion 2) camera->base shift 3) user-configurable offsets
            base_position = relative_pos_ros - self.camera_to_base_offset
            base_position = base_position.copy()
            base_position[2] += self.get_lift_z_delta_for_arm_pose()
            base_position, base_rotation = self.apply_wrist_offsets(
                side, base_position, relative_rot_ros
            )
            arm_quaternion = base_rotation.as_quat()  # [x, y, z, w]

            wrist_pose = PoseStamped()
            wrist_pose.header.stamp = self.get_clock().now().to_msg()
            wrist_pose.header.frame_id = 'base_link'
            wrist_pose.pose.position = self.safe_point(
                base_position[0], base_position[1], base_position[2]
            )
            wrist_pose.pose.orientation = self.safe_quaternion(
                arm_quaternion[0], arm_quaternion[1], arm_quaternion[2], arm_quaternion[3]
            )

            if side == 'left':
                self.left_wrist_rviz_pub.publish(wrist_pose)
            elif side == 'right':
                self.right_wrist_rviz_pub.publish(wrist_pose)
            self.last_pose_publish_sec[pose_key] = now_sec

        except Exception as e:
            self.get_logger().warn(f'Error publishing wrist pose from matrix for {side}: {e}')

    def _publish_elbow_pose_from_matrix(self, world_joint_matrix, side):
        """Publish elbow pose from a body joint world matrix."""
        try:
            if not self.can_publish_goal_pose():
                return
            pose_key = f'{side}_elbow'
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (self.pose_min_period > 0.0 and
                    (now_sec - self.last_pose_publish_sec[pose_key])
                    < self.pose_min_period):
                return

            relative_joint_matrix = self.head_inverse_matrix @ world_joint_matrix
            relative_pos_head, relative_quat_head = self.matrix_to_pose(
                relative_joint_matrix
            )
            relative_pos_ros, relative_quat_ros = self.vr_to_ros_transform(
                relative_pos_head, relative_quat_head
            )
            relative_pos_ros = self.scale_goal_position(relative_pos_ros)
            elbow_rotation = R.from_quat(relative_quat_ros)

            base_position = relative_pos_ros - self.camera_to_base_offset
            base_position = base_position.copy()
            base_position[2] += self.get_lift_z_delta_for_arm_pose()
            side_key = side if side in ('left', 'right') else 'left'
            base_position = base_position + self.elbow_position_offsets[side_key]
            elbow_quaternion = elbow_rotation.as_quat()

            elbow_pose = PoseStamped()
            elbow_pose.header.stamp = self.get_clock().now().to_msg()
            elbow_pose.header.frame_id = 'base_link'
            elbow_pose.pose.position = self.safe_point(
                base_position[0], base_position[1], base_position[2]
            )
            elbow_pose.pose.orientation = self.safe_quaternion(
                elbow_quaternion[0], elbow_quaternion[1], elbow_quaternion[2], elbow_quaternion[3]
            )

            if side == 'left':
                self.left_elbow_rviz_pub.publish(elbow_pose)
            elif side == 'right':
                self.right_elbow_rviz_pub.publish(elbow_pose)
            self.last_pose_publish_sec[pose_key] = now_sec

        except Exception as e:
            self.get_logger().warn(f'Error publishing elbow pose from matrix for {side}: {e}')

    def process_thumbstick(self):
        """Process thumbstick input for mode switching and joystick control."""
        try:
            left_thumbstick_pressed = False
            right_thumbstick_pressed = False
            left_thumbstick_value = [0.0, 0.0]
            right_thumbstick_value = [0.0, 0.0]

            if isinstance(self.left_controller_state, dict):
                left_thumbstick_pressed = bool(self.left_controller_state.get('thumbstick', False))
                thumbstick_val = self.left_controller_state.get('thumbstickValue', [0.0, 0.0])
                if isinstance(thumbstick_val, (list, tuple)) and len(thumbstick_val) >= 2:
                    lx = float(thumbstick_val[0])
                    ly = float(thumbstick_val[1])
                    if self.left_stick_swap_xy:
                        lx, ly = ly, lx
                    if self.left_reverse_x:
                        lx = -lx
                    if self.left_reverse_y:
                        ly = -ly
                    left_thumbstick_value = [lx, ly]

            if isinstance(self.right_controller_state, dict):
                right_thumbstick_pressed = bool(
                    self.right_controller_state.get('thumbstick', False)
                )
                thumbstick_val = self.right_controller_state.get('thumbstickValue', [0.0, 0.0])
                if isinstance(thumbstick_val, (list, tuple)) and len(thumbstick_val) >= 2:
                    rx = float(thumbstick_val[0])
                    ry = float(thumbstick_val[1])
                    if self.right_stick_swap_xy:
                        rx, ry = ry, rx
                    # Fixed convention: invert right stick X sign.
                    rx = -rx
                    right_thumbstick_value = [rx, ry]

            # Toggle mode on both-thumbstick click rising edge.
            if left_thumbstick_pressed and right_thumbstick_pressed:
                if not self.prev_left_thumbstick_pressed or not self.prev_right_thumbstick_pressed:
                    self.joystick_mode = not self.joystick_mode
                    mode_name = 'LIFT+HEAD' if self.joystick_mode else 'LIFT+CMD_VEL'
                    self.get_logger().info(f'[THUMBSTICK] Mode switched to: {mode_name}')
                    if self.joystick_mode:
                        # Ensure base stops when leaving cmd_vel mode.
                        self.publish_cmd_vel_from_thumbstick([0.0, 0.0], [0.0, 0.0])

            self.prev_left_thumbstick_pressed = left_thumbstick_pressed
            self.prev_right_thumbstick_pressed = right_thumbstick_pressed

            # Lift always follows right Y axis.
            # Match joystick_controller: lift uses right X axis.
            if abs(right_thumbstick_value[0]) > 0.0:
                self.publish_right_joystick(right_thumbstick_value[0])

            # Left stick controls head in joystick_mode, otherwise base cmd_vel.
            if self.joystick_mode:
                if abs(left_thumbstick_value[0]) > 0.0 or abs(left_thumbstick_value[1]) > 0.0:
                    self.publish_left_joystick_from_thumbstick(left_thumbstick_value)
            else:
                self.publish_cmd_vel_from_thumbstick(left_thumbstick_value, right_thumbstick_value)

        except Exception as e:
            self.get_logger().error(f'Error processing thumbstick: {e}')

    def publish_right_joystick(self, thumbstick_value):
        """Publish lift_joint target from right thumbstick."""
        try:
            deadzone_applied_value = self.apply_deadzone(float(thumbstick_value))
            if abs(deadzone_applied_value) <= 1e-6:
                return

            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (now_sec - self.last_lift_publish_sec) < self.control_min_period:
                return

            # Integrate on the last commanded value so stick input accumulates
            # even when /joint_states feedback is slower than controller events.
            base_lift_position = (
                self.last_lift_command
                if self.last_lift_command is not None
                else self.lift_joint_current_position
            )
            new_lift_position = base_lift_position + deadzone_applied_value * self.right_jog_scale

            msg = JointTrajectory()
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.header.frame_id = ''
            msg.joint_names = ['lift_joint']

            point = JointTrajectoryPoint()
            point.positions = [new_lift_position]
            point.velocities = [0.0]
            point.accelerations = [0.0]
            point.effort = []
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 0

            msg.points.append(point)
            self.lift_joint_pub.publish(msg)
            self.last_lift_publish_sec = now_sec
            self.last_lift_command = new_lift_position

        except Exception as e:
            self.get_logger().error(f'Error publishing right joystick: {e}')

    def publish_left_joystick_from_thumbstick(self, thumbstick_value):
        """Publish head joints target from left thumbstick."""
        try:
            deadzone_applied_x = self.apply_deadzone(float(thumbstick_value[0]))
            deadzone_applied_y = self.apply_deadzone(float(thumbstick_value[1]))
            if abs(deadzone_applied_x) <= 1e-6 and abs(deadzone_applied_y) <= 1e-6:
                return

            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (now_sec - self.last_head_publish_sec) < self.control_min_period:
                return

            # Integrate on the last commanded value so stick input accumulates
            # even when /joint_states feedback is slower than controller events.
            if self.last_head_command is not None:
                base_head_joint1_position, base_head_joint2_position = self.last_head_command
            else:
                base_head_joint1_position = self.head_joint1_current_position
                base_head_joint2_position = self.head_joint2_current_position

            new_head_joint1_position = (
                base_head_joint1_position
                + deadzone_applied_x * self.left_jog_scale
            )
            new_head_joint2_position = (
                base_head_joint2_position
                + deadzone_applied_y * self.left_jog_scale
            )

            msg = JointTrajectory()
            msg.joint_names = ['head_joint1', 'head_joint2']

            point = JointTrajectoryPoint()
            point.positions = [new_head_joint1_position, new_head_joint2_position]
            point.velocities = [0.0, 0.0]
            point.accelerations = [0.0, 0.0]
            point.effort = []
            msg.points.append(point)

            self.head_joint_pub.publish(msg)
            self.last_head_publish_sec = now_sec
            self.last_head_command = (new_head_joint1_position, new_head_joint2_position)

        except Exception as e:
            self.get_logger().error(f'Error publishing left joystick from thumbstick: {e}')

    def publish_cmd_vel_from_thumbstick(self, left_thumbstick_value, right_thumbstick_value):
        """Publish base cmd_vel from thumbstick values."""
        try:
            if not self.vr_publishing_enabled:
                return

            left_x_deadzone = self.apply_deadzone(float(left_thumbstick_value[0]))
            left_y_deadzone = self.apply_deadzone(float(left_thumbstick_value[1]))
            right_y_deadzone = self.apply_deadzone(float(right_thumbstick_value[1]))

            twist_msg = Twist()
            # Apply requested sign convention for SG2 base linear axes.
            twist_msg.linear.x = -left_x_deadzone / self.linear_x_scale
            twist_msg.linear.y = left_y_deadzone / self.linear_y_scale
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = -right_y_deadzone / self.angular_z_scale

            cmd_tuple = (twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z)
            is_same_command = (
                abs(cmd_tuple[0] - self.last_cmd_vel_command[0]) < 1e-5 and
                abs(cmd_tuple[1] - self.last_cmd_vel_command[1]) < 1e-5 and
                abs(cmd_tuple[2] - self.last_cmd_vel_command[2]) < 1e-5
            )

            now_sec = self.get_clock().now().nanoseconds / 1e9
            # Keep sending at limited rate while moving, but suppress identical zero spam.
            if (is_same_command and abs(cmd_tuple[0]) < 1e-6
                    and abs(cmd_tuple[1]) < 1e-6
                    and abs(cmd_tuple[2]) < 1e-6):
                return
            if ((now_sec - self.last_cmd_vel_publish_sec) < self.control_min_period
                    and is_same_command):
                return

            self.cmd_vel_pub.publish(twist_msg)
            self.last_cmd_vel_publish_sec = now_sec
            self.last_cmd_vel_command = cmd_tuple

        except Exception as e:
            self.get_logger().error(f'Error publishing cmd_vel from thumbstick: {e}')

    def transform_and_publish_pose(self, pose_array_msg, publisher, hand_name, vr_scale=1.0):
        """Transform pose from head relative coordinates to base_link and publish."""
        if not self.can_publish_goal_pose():
            return
        if not pose_array_msg.poses:
            return

        # Assume the first pose in the array is the wrist pose (head relative, ROS coordinates)
        wrist_pose_relative = pose_array_msg.poses[0]

        # Extract relative position (head/camera relative, already in ROS coordinate system)
        camera_relative_position = np.array([
            wrist_pose_relative.position.x * vr_scale,
            wrist_pose_relative.position.y * vr_scale,
            wrist_pose_relative.position.z * vr_scale
        ], dtype=np.float64)
        camera_relative_position = self.scale_goal_position(camera_relative_position)

        # Extract relative orientation (head/camera relative, already in ROS coordinate system)
        camera_relative_quaternion = np.array([
            wrist_pose_relative.orientation.x,
            wrist_pose_relative.orientation.y,
            wrist_pose_relative.orientation.z,
            wrist_pose_relative.orientation.w
        ], dtype=np.float64)

        # Transform from camera relative coordinates directly to base_link coordinates
        base_position = camera_relative_position - self.camera_to_base_offset
        base_position = base_position.copy()
        base_position[2] += self.get_lift_z_delta_for_arm_pose()

        # Use camera relative orientation as is
        camera_relative_rotation = R.from_quat(camera_relative_quaternion)
        base_position, camera_relative_rotation = self.apply_wrist_offsets(
            hand_name, base_position, camera_relative_rotation
        )
        arm_quaternion = camera_relative_rotation.as_quat()  # [x, y, z, w]

        # Create target pose message
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'base_link'

        # Small offset for better visualization
        target_pose.pose.position.x = base_position[0] - 0.15
        target_pose.pose.position.y = base_position[1]
        target_pose.pose.position.z = base_position[2]

        target_pose.pose.orientation.x = arm_quaternion[0]
        target_pose.pose.orientation.y = arm_quaternion[1]
        target_pose.pose.orientation.z = arm_quaternion[2]
        target_pose.pose.orientation.w = arm_quaternion[3]

        publisher.publish(target_pose)

        self.get_logger().debug(
            'Transformed {} pose: pos=[{:.3f}, {:.3f}, {:.3f}]'.format(
                hand_name, base_position[0], base_position[1], base_position[2]
            )
        )

    def start_vuer_server(self):
        """Start the VR server in a separate thread."""
        def run_server():
            try:
                asyncio.set_event_loop(self.loop)
                self.get_logger().info('Starting VR server...')
                # spawn(start=True) internally starts and runs the server loop.
                self.vuer.spawn(start=True)(self.main_hand_tracking)
            except Exception as e:
                self.get_logger().error(f'Error in VR server thread: {e}')

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    async def main_hand_tracking(self, session):
        """Run main controller/body tracking session."""
        try:
            fps = self.fps
            self.get_logger().info('Starting controller/body tracking session')
            session.upsert(
                MotionControllers(
                    stream=True,
                    key='motion-controller',
                    left=True,
                    right=True,
                ),
                to='bgChildren',
            )
            session.upsert(
                Body(
                    fps=fps,
                    stream=True,
                    key='body_tracking',
                    leftHand=False,
                    rightHand=False,
                    hideIndicate=False,
                    showFrame=True,
                    showBody=True,
                    frameScale=0.02,
                ),
                to='children',
            )
            self.get_logger().info('Controller and body tracking enabled')
            while True:
                await asyncio.sleep(1/fps)
        except Exception as e:
            self.get_logger().error(f'Error in controller/body tracking session: {e}')

    async def on_body_tracking_move(self, event, session):
        """Handle body tracking events and update head transform for controller-relative pose."""
        try:
            if not self.vr_publishing_enabled:
                return
            if not isinstance(event.value, dict):
                return

            body_data = event.value.get('body')
            if not isinstance(body_data, (list, tuple, np.ndarray)):
                return

            body_array = (
                body_data if isinstance(body_data, np.ndarray)
                else np.asarray(body_data, dtype=np.float64)
            )
            start_idx = BODY_HEAD_INDEX * 16
            end_idx = start_idx + 16
            if body_array.size < end_idx:
                return

            head_matrix = np.asarray(
                body_array[start_idx:end_idx], dtype=np.float64
            ).reshape(4, 4, order='F')
            if not np.all(np.isfinite(head_matrix)):
                return

            # Reject degenerate head matrices from uninitialized body tracking.
            if abs(float(np.linalg.det(head_matrix[:3, :3]))) < 1e-6:
                return

            self.head_transform_matrix = head_matrix
            try:
                self.head_inverse_matrix = np.linalg.inv(head_matrix)
            except np.linalg.LinAlgError:
                return

            left_elbow_matrix = self.get_body_joint_matrix_from_flat(
                body_array, BODY_LEFT_ELBOW_INDEX
            )
            if left_elbow_matrix is not None:
                self._publish_elbow_pose_from_matrix(left_elbow_matrix, 'left')

            right_elbow_matrix = self.get_body_joint_matrix_from_flat(
                body_array, BODY_RIGHT_ELBOW_INDEX
            )
            if right_elbow_matrix is not None:
                self._publish_elbow_pose_from_matrix(right_elbow_matrix, 'right')

        except Exception as e:
            self.get_logger().error(f'Error in body tracking event: {e}')

    async def on_controller_move(self, event, session):
        """Handle Meta Quest controller events (CONTROLLER_MOVE)."""
        try:
            if not self.vr_publishing_enabled:
                return
            if not isinstance(event.value, dict):
                return

            data = event.value

            left_state = data.get('leftState')
            if isinstance(left_state, dict):
                self.left_controller_state = left_state
                squeeze_val = left_state.get('squeezeValue')
                if self.is_valid_float(squeeze_val):
                    self.left_squeeze_value = float(squeeze_val)
                    left_squeeze_msg = Float32()
                    left_squeeze_msg.data = self.left_squeeze_value
                    self.left_squeeze_pub.publish(left_squeeze_msg)
                else:
                    self.left_squeeze_value = 0.0

                trigger_val = left_state.get('triggerValue')
                if self.is_valid_float(trigger_val):
                    calibrated_trigger = self.calibrate_trigger('left', trigger_val)
                    left_trigger_msg = Float32()
                    left_trigger_msg.data = calibrated_trigger
                    self.left_trigger_pub.publish(left_trigger_msg)
            else:
                self.left_squeeze_value = 0.0

            right_state = data.get('rightState')
            if isinstance(right_state, dict):
                self.right_controller_state = right_state
                squeeze_val = right_state.get('squeezeValue')
                if self.is_valid_float(squeeze_val):
                    self.right_squeeze_value = float(squeeze_val)
                    right_squeeze_msg = Float32()
                    right_squeeze_msg.data = self.right_squeeze_value
                    self.right_squeeze_pub.publish(right_squeeze_msg)
                else:
                    self.right_squeeze_value = 0.0

                trigger_val = right_state.get('triggerValue')
                if self.is_valid_float(trigger_val):
                    calibrated_trigger = self.calibrate_trigger('right', trigger_val)
                    right_trigger_msg = Float32()
                    right_trigger_msg.data = calibrated_trigger
                    self.right_trigger_pub.publish(right_trigger_msg)
            else:
                self.right_squeeze_value = 0.0

            # Process thumbstick for lift/head/cmd_vel control.
            self.process_thumbstick()

            # Call reactivate when both A buttons are pressed (rising edge only)
            left_a = (
                bool(self.left_controller_state.get('aButton', False))
                if isinstance(self.left_controller_state, dict) else False
            )
            right_a = (
                bool(self.right_controller_state.get('aButton', False))
                if isinstance(self.right_controller_state, dict) else False
            )
            both_a_now = left_a and right_a
            if both_a_now and not self.both_a_buttons_pressed_prev:
                self._call_reactivate()
            self.both_a_buttons_pressed_prev = both_a_now

            left_matrix_raw = data.get('left')
            if isinstance(left_matrix_raw, (list, np.ndarray)) and len(left_matrix_raw) == 16:
                self.left_controller_matrix = np.asarray(
                    left_matrix_raw, dtype=np.float64
                ).reshape(4, 4, order='F')
                self._publish_wrist_pose_from_matrix(self.left_controller_matrix, 'left')

            right_matrix_raw = data.get('right')
            if isinstance(right_matrix_raw, (list, np.ndarray)) and len(right_matrix_raw) == 16:
                self.right_controller_matrix = np.asarray(
                    right_matrix_raw, dtype=np.float64
                ).reshape(4, 4, order='F')
                self._publish_wrist_pose_from_matrix(self.right_controller_matrix, 'right')

            self.controller_log_counter += 1
            if self.controller_log_counter % self.log_every_n == 0:
                l_trg_raw = (
                    float(self.left_controller_state.get('triggerValue', 0.0))
                    if isinstance(self.left_controller_state, dict) else 0.0
                )
                r_trg_raw = (
                    float(self.right_controller_state.get('triggerValue', 0.0))
                    if isinstance(self.right_controller_state, dict) else 0.0
                )
                l_trg = self.calibrate_trigger('left', l_trg_raw)
                r_trg = self.calibrate_trigger('right', r_trg_raw)
                l_stick = (
                    self.left_controller_state.get('thumbstickValue', [0.0, 0.0])
                    if isinstance(self.left_controller_state, dict)
                    else [0.0, 0.0]
                )
                r_stick = (
                    self.right_controller_state.get('thumbstickValue', [0.0, 0.0])
                    if isinstance(self.right_controller_state, dict)
                    else [0.0, 0.0]
                )
                self.get_logger().info(
                    f'Controller data received | '
                    f'left_matrix={self.left_controller_matrix is not None}, '
                    f'right_matrix={self.right_controller_matrix is not None}, '
                    f'left_trigger_raw={l_trg_raw:.3f}, right_trigger_raw={r_trg_raw:.3f}, '
                    f'left_trigger={l_trg:.3f}, right_trigger={r_trg:.3f}, '
                    f'left_stick={l_stick}, right_stick={r_stick}, '
                    f'mode={"LIFT+HEAD" if self.joystick_mode else "LIFT+CMD_VEL"}'
                )

        except Exception as e:
            self.get_logger().error(f'Error in controller move event: {e}')

    def __del__(self):
        try:
            if hasattr(self, 'vuer'):
                self.loop.run_until_complete(self.vuer.stop())
            if hasattr(self, 'loop'):
                self.loop.close()
        except Exception as e:
            self.get_logger().error(f'Error in cleanup: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VRTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
