#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
import math
import os
import socket
import threading
import traceback

from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
import nest_asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from robotis_interfaces.msg import HandJoints
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vuer import Vuer
from vuer.schemas import Body, Hands, ImageBackground

# Allow nested asyncio execution
nest_asyncio.apply()

# WebXR Body Tracking joint order (XRBodyJoint enum)
BODY_JOINT_KEYS = [
    'hips',
    'spine-lower',
    'spine-middle',
    'spine-upper',
    'chest',
    'neck',
    'head',
    'left-shoulder',
    'left-scapula',
    'left-arm-upper',
    'left-arm-lower',
    'left-hand-wrist-twist',
    'right-shoulder',
    'right-scapula',
    'right-arm-upper',
    'right-arm-lower',
    'right-hand-wrist-twist',
    'left-hand-palm',
    'left-hand-wrist',
    'left-hand-thumb-metacarpal',
    'left-hand-thumb-phalanx-proximal',
    'left-hand-thumb-phalanx-distal',
    'left-hand-thumb-tip',
    'left-hand-index-metacarpal',
    'left-hand-index-phalanx-proximal',
    'left-hand-index-phalanx-intermediate',
    'left-hand-index-phalanx-distal',
    'left-hand-index-tip',
    'left-hand-middle-phalanx-metacarpal',
    'left-hand-middle-phalanx-proximal',
    'left-hand-middle-phalanx-intermediate',
    'left-hand-middle-phalanx-distal',
    'left-hand-middle-tip',
    'left-hand-ring-metacarpal',
    'left-hand-ring-phalanx-proximal',
    'left-hand-ring-phalanx-intermediate',
    'left-hand-ring-phalanx-distal',
    'left-hand-ring-tip',
    'left-hand-little-metacarpal',
    'left-hand-little-phalanx-proximal',
    'left-hand-little-phalanx-intermediate',
    'left-hand-little-phalanx-distal',
    'left-hand-little-tip',
    'right-hand-palm',
    'right-hand-wrist',
    'right-hand-thumb-metacarpal',
    'right-hand-thumb-phalanx-proximal',
    'right-hand-thumb-phalanx-distal',
    'right-hand-thumb-tip',
    'right-hand-index-metacarpal',
    'right-hand-index-phalanx-proximal',
    'right-hand-index-phalanx-intermediate',
    'right-hand-index-phalanx-distal',
    'right-hand-index-tip',
    'right-hand-middle-metacarpal',
    'right-hand-middle-phalanx-proximal',
    'right-hand-middle-phalanx-intermediate',
    'right-hand-middle-phalanx-distal',
    'right-hand-middle-tip',
    'right-hand-ring-metacarpal',
    'right-hand-ring-phalanx-proximal',
    'right-hand-ring-phalanx-intermediate',
    'right-hand-ring-phalanx-distal',
    'right-hand-ring-tip',
    'right-hand-little-metacarpal',
    'right-hand-little-phalanx-proximal',
    'right-hand-little-phalanx-intermediate',
    'right-hand-little-phalanx-distal',
    'right-hand-little-tip',
    'left-upper-leg',
    'left-lower-leg',
    'left-foot-ankle-twist',
    'left-foot-ankle',
    'left-foot-subtalar',
    'left-foot-transverse',
    'left-foot-ball',
    'right-upper-leg',
    'right-lower-leg',
    'right-foot-ankle-twist',
    'right-foot-ankle',
    'right-foot-subtalar',
    'right-foot-transverse',
    'right-foot-ball',
]


class VRTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('vr_trajectory_publisher')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Lift and base (whole-body) parameters
        self.declare_parameter('enable_lift_publishing', True)
        self.declare_parameter('enable_head_publishing', False)
        self.declare_parameter('enable_base_publishing', True)
        self.declare_parameter('enable_vr_image', False)

        self.declare_parameter('base_linear_kp', 1.7)
        self.declare_parameter('base_angular_kp', 2.0)
        self.declare_parameter('base_linear_deadzone', 0.1)
        self.declare_parameter('base_angular_deadzone', 0.05)
        self.declare_parameter('base_max_linear_velocity', 0.3)
        self.declare_parameter('base_max_angular_velocity', 0.5)
        self.declare_parameter('enable_base_debug_topics', False)
        self.declare_parameter('base_divergence_position_threshold', 0.5)
        self.declare_parameter('base_divergence_yaw_threshold', 0.5)

        self.enable_lift_publishing = (
            self.get_parameter('enable_lift_publishing')
            .get_parameter_value().bool_value
        )
        self.enable_head_publishing = (
            self.get_parameter('enable_head_publishing')
            .get_parameter_value().bool_value
        )
        self.enable_base_publishing = (
            self.get_parameter('enable_base_publishing')
            .get_parameter_value().bool_value
        )
        self.base_linear_kp = (
            self.get_parameter('base_linear_kp')
            .get_parameter_value().double_value
        )
        self.base_angular_kp = (
            self.get_parameter('base_angular_kp')
            .get_parameter_value().double_value
        )
        self.base_linear_deadzone = (
            self.get_parameter('base_linear_deadzone')
            .get_parameter_value().double_value
        )
        self.base_angular_deadzone = (
            self.get_parameter('base_angular_deadzone')
            .get_parameter_value().double_value
        )
        self.base_max_linear_velocity = (
            self.get_parameter('base_max_linear_velocity')
            .get_parameter_value().double_value
        )
        self.base_max_angular_velocity = (
            self.get_parameter('base_max_angular_velocity')
            .get_parameter_value().double_value
        )
        self.enable_base_debug_topics = (
            self.get_parameter('enable_base_debug_topics')
            .get_parameter_value().bool_value
        )
        self.base_divergence_position_threshold = (
            self.get_parameter('base_divergence_position_threshold')
            .get_parameter_value().double_value
        )
        self.base_divergence_yaw_threshold = (
            self.get_parameter('base_divergence_yaw_threshold')
            .get_parameter_value().double_value
        )

        if self.base_linear_kp <= 0.0:
            self.get_logger().warn('base_linear_kp must be positive. Using default 2.0.')
            self.base_linear_kp = 2.0
        if self.base_angular_kp <= 0.0:
            self.get_logger().warn('base_angular_kp must be positive. Using default 1.0.')
            self.base_angular_kp = 1.0
        if self.base_linear_deadzone < 0.0:
            self.base_linear_deadzone = 0.05
        if self.base_angular_deadzone < 0.0:
            self.base_angular_deadzone = 0.05

        # VR image in headset (stereo background from camera topics)
        self.declare_parameter(
            'vr_image_left_topic',
            '/zed/zed_node/left/image_rect_color/compressed'
        )
        self.declare_parameter(
            'vr_image_right_topic',
            '/zed/zed_node/right/image_rect_color/compressed'
        )
        self.declare_parameter('vr_image_fps', 15.0)

        # Wrist/elbow position offsets (head-relative, ROS frame: X forward, Y left, Z up)
        self.declare_parameter('wrist_offset_x', 0.0)
        self.declare_parameter('wrist_offset_y', 0.0)
        self.declare_parameter('wrist_offset_z', 0.0)
        self.declare_parameter('elbow_offset_x', 0.0)
        self.declare_parameter('elbow_offset_y', 0.0)
        self.declare_parameter('elbow_offset_z', 0.0)

        self.enable_vr_image = (
            self.get_parameter('enable_vr_image')
            .get_parameter_value().bool_value
        )
        self.vr_image_left_topic = (
            self.get_parameter('vr_image_left_topic')
            .get_parameter_value().string_value
        )
        self.vr_image_right_topic = (
            self.get_parameter('vr_image_right_topic')
            .get_parameter_value().string_value
        )
        self.vr_image_fps = self.get_parameter('vr_image_fps').get_parameter_value().double_value
        self.wrist_offsets = {
            'x': self.get_parameter('wrist_offset_x').get_parameter_value().double_value,
            'y': self.get_parameter('wrist_offset_y').get_parameter_value().double_value,
            'z': self.get_parameter('wrist_offset_z').get_parameter_value().double_value,
        }
        self.elbow_offsets = {
            'x': self.get_parameter('elbow_offset_x').get_parameter_value().double_value,
            'y': self.get_parameter('elbow_offset_y').get_parameter_value().double_value,
            'z': self.get_parameter('elbow_offset_z').get_parameter_value().double_value,
        }

        self.get_logger().info(
            f'Whole-body: lift={self.enable_lift_publishing}, base={self.enable_base_publishing}, '
            f'base_linear_kp={self.base_linear_kp}, base_angular_kp={self.base_angular_kp}, '
            f'base_debug_topics={self.enable_base_debug_topics}, vr_image={self.enable_vr_image}'
        )

        # VR publishing control flag
        self.vr_publishing_enabled = False

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

        try:
            import vuer
            self.get_logger().info(f'Vuer version: {vuer.__version__}')
        except AttributeError:
            self.get_logger().info('Vuer version: (not set)')

        self.fps = 30
        self.get_logger().info(f'VR Trajectory server available at: https://{hostname}:8012')

        # VR event handlers
        self.vuer.add_handler('HAND_MOVE')(self.on_hand_move)
        self.vuer.add_handler('BODY_MOVE')(self.on_body_tracking_move)

        # QoS setting
        self.vr_stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.left_hand_pos_pub = self.create_publisher(
            HandJoints,
            '/left_hand/hand_joint_pos',
            self.vr_stream_qos
        )
        self.right_hand_pos_pub = self.create_publisher(
            HandJoints,
            '/right_hand/hand_joint_pos',
            self.vr_stream_qos
        )
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
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', self.vr_stream_qos
        )
        self.base_divergence_pub = self.create_publisher(
            Bool, '/vr_base_divergence', self.vr_stream_qos
        )
        self.vr_camera_goal_pub = self.create_publisher(
            PoseStamped, '/vr_camera_goal_pose', self.vr_stream_qos
        )
        self.left_wrist_rviz_pub = self.create_publisher(
            PoseStamped, '/l_goal_pose', self.vr_stream_qos
        )
        self.right_wrist_rviz_pub = self.create_publisher(
            PoseStamped, '/r_goal_pose', self.vr_stream_qos
        )
        self.left_elbow_pub = self.create_publisher(
            PoseStamped, '/l_elbow_pose', self.vr_stream_qos
        )
        self.right_elbow_pub = self.create_publisher(
            PoseStamped, '/r_elbow_pose', self.vr_stream_qos
        )

        # Subscriber for VR control toggle
        self.vr_control_sub = self.create_subscription(
            Bool,
            '/vr_control/toggle',
            self.vr_control_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, self.vr_stream_qos
        )

        # VR image (stereo background): optional, enabled by enable_vr_image
        self.current_session = None
        self.latest_left_bytes = None
        self.latest_right_bytes = None
        if self.enable_vr_image:
            self.left_image_sub = self.create_subscription(
                CompressedImage,
                self.vr_image_left_topic,
                self.left_image_callback,
                self.vr_stream_qos,
            )
            self.right_image_sub = self.create_subscription(
                CompressedImage,
                self.vr_image_right_topic,
                self.right_image_callback,
                self.vr_stream_qos,
            )
            period = 1.0 / self.vr_image_fps if self.vr_image_fps > 0 else 1.0 / 15.0
            self.image_send_timer = self.create_timer(period, self.send_latest_images)
            self.get_logger().info(
                f'VR image enabled: left={self.vr_image_left_topic}, '
                f'right={self.vr_image_right_topic}, {self.vr_image_fps} fps'
            )

        self.required_vr_frames = [0,
                                   1, 2, 3, 4,
                                   6, 7, 8, 9,
                                   11, 12, 13, 14,
                                   16, 17, 18, 19,
                                   21, 22, 23, 24]

        self.vr_hand_to_urdf = np.array([
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]
        ])

        self.prev_poses_right = np.zeros((21, 3))
        self.start_poses_right = False

        self.prev_poses_left = np.zeros((21, 3))
        self.start_poses_left = False

        # Cached transform constants for hot paths
        self.vr_to_ros_matrix = np.array([
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ], dtype=np.float64)
        self.body_head_to_ros_rot = R.from_matrix(self.BODY_HEAD_TO_ROS_POSITION)
        self.zedm_to_base_offset = np.array([
            0.0 - 0.0238122 - 0.040 - 0.049483 - 0.0055,
            0.0 + 0.0 + 0.0 + 0.0 + 0.0,
            -0.01325 + 0.0242094 - 0.054 - 0.102130 - 1.4316,
        ], dtype=np.float64)

        # VR data storage
        self.left_hand_data = None
        self.right_hand_data = None
        self.head_transform_matrix = np.eye(4)
        self.head_inverse_matrix = np.eye(4)
        # head orientation in ROS frame for base_link pose
        self.head_ros_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self.hand_pose_is_head_relative = self.declare_parameter(
            'hand_pose_is_head_relative', True
        ).value
        self.zero_z_on_start = self.declare_parameter(
            'zero_z_on_start', False
        ).value
        self.apply_head_height_to_arm_z = self.declare_parameter(
            'apply_head_height_to_arm_z', True
        ).value
        self.z_calibrated = False
        self.z_calibration_offset = 0.0
        self.head_height_offset_for_arms = 0.0

        # Whole-body: camera/odom reference for lift and base
        self.initial_camera_height = None
        self.initial_camera_position = None
        self.initial_camera_yaw = None
        self.previous_camera_position = None
        self.previous_camera_yaw = None
        self.current_odom = None
        self.initial_odom_position = None
        self.initial_odom_yaw = None
        self.cmd_vel_log_counter = 0
        self.cmd_vel_log_every_n = 10
        self.base_divergence_log_counter = 0
        self.base_divergence_log_every_n = 20

        # Low-pass filter settings (wrist/elbow pose)
        self.low_pass_filter_alpha = 0.9
        self.pose_filters = {}

        # Lift low-pass filter (smooths height command) and max velocity
        # (m/s, same units as lift position)
        self.declare_parameter('lift_low_pass_alpha', 0.3)
        self.declare_parameter('max_lift_velocity', 0.07)
        self.lift_low_pass_alpha = (
            self.get_parameter('lift_low_pass_alpha')
            .get_parameter_value().double_value
        )
        self.max_lift_velocity = (
            self.get_parameter('max_lift_velocity')
            .get_parameter_value().double_value
        )
        self.filtered_lift_position = None
        self.last_lift_time = None
        self.max_elbow_wrist_distance = 0.4
        self.max_wrist_angle_step_deg = 30.0

        # Scaling VR data
        self.scaling_vr = 1.1
        self.wrist_vr_scale = 1.4
        self.elbow_vr_scale = 1.4

        # Head pitch offset configuration
        self.pitch_offset = -0.5

        self.hand_log_counter = 0
        self.wrist_debug_log_counter = 0
        self.wrist_debug_log_every_n = 30

        self.status_timer = self.create_timer(5.0, self.log_status)
        self.head_log_counter = 0
        self.log_every_n = self.fps

        # Async event loop for Vuer server
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.start_vuer_server()

        self.get_logger().info('VR Trajectory Publisher node has been started')
        vr_status = 'ENABLED' if self.vr_publishing_enabled else 'DISABLED'
        self.get_logger().info(
            f'VR publishing is {vr_status} by default. '
            f'Send /vr_control/toggle message (True=enable, False=disable).'
        )

    def odom_callback(self, msg):
        """Receive robot odometry for base control."""
        self.current_odom = msg

    def vr_control_callback(self, msg):
        """Enable/disable VR publishing based on message content."""
        new_state = bool(msg.data)

        if new_state != self.vr_publishing_enabled:
            self.vr_publishing_enabled = new_state
            status = 'ENABLED' if self.vr_publishing_enabled else 'DISABLED'
            self.get_logger().info(
                f'VR publishing changed to: {status} (message value: {msg.data})'
            )

            if self.vr_publishing_enabled:
                self.start_poses_left = False
                self.start_poses_right = False
                self.prev_poses_left.fill(0.0)
                self.prev_poses_right.fill(0.0)
                self.initial_camera_height = None
                self.initial_camera_position = None
                self.initial_camera_yaw = None
                self.previous_camera_position = None
                self.previous_camera_yaw = None
                if self.current_odom is not None:
                    pos = self.current_odom.pose.pose.position
                    quat = self.current_odom.pose.pose.orientation
                    r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
                    _, _, yaw = r.as_euler('xyz')
                    self.initial_odom_position = np.array([pos.x, pos.y])
                    self.initial_odom_yaw = yaw
                    self.get_logger().info(
                        f'Initial robot odom position: '
                        f'[{self.initial_odom_position[0]:.3f}, '
                        f'{self.initial_odom_position[1]:.3f}], '
                        f'yaw: {self.initial_odom_yaw:.3f} rad'
                    )
                else:
                    self.initial_odom_position = None
                    self.initial_odom_yaw = None
                    self.get_logger().warn('VR control enabled but odom not available yet')

            if not self.vr_publishing_enabled:
                self.left_joint_positions = [0.0] * 20
                self.right_joint_positions = [0.0] * 20
                self.start_poses_left = False
                self.start_poses_right = False
                self.prev_poses_left.fill(0.0)
                self.prev_poses_right.fill(0.0)
                self.initial_odom_position = None
                self.initial_odom_yaw = None
                self.initial_camera_height = None
                self.initial_camera_position = None
                self.initial_camera_yaw = None
                self.filtered_lift_position = None
                self.last_lift_time = None
                self.z_calibrated = False
                self.head_height_offset_for_arms = 0.0
                self.pose_filters.clear()
                self.get_logger().info('Joint positions reset to zero')

    def log_status(self):
        """Log current system status for debugging."""
        vr_status = 'ENABLED' if self.vr_publishing_enabled else 'DISABLED'
        self.get_logger().info(f'Status: VR={vr_status}')

    def left_image_callback(self, msg):
        """Store latest left eye image for VR background (only when enable_vr_image)."""
        if not self.enable_vr_image:
            return
        self.latest_left_bytes = bytes(msg.data)

    def right_image_callback(self, msg):
        """Store latest right eye image for VR background (only when enable_vr_image)."""
        if not self.enable_vr_image:
            return
        self.latest_right_bytes = bytes(msg.data)

    def send_latest_images(self):
        """Timer callback: send latest stereo frames to Vuer at configured fps."""
        if not self.enable_vr_image or not self.vr_publishing_enabled:
            return
        if not self.loop.is_running():
            return
        if self.latest_left_bytes is not None:
            img = self.latest_left_bytes
            self.latest_left_bytes = None
            asyncio.run_coroutine_threadsafe(
                self.update_vuer_background(img, key='bg_left', layer=1),
                self.loop,
            )
        if self.latest_right_bytes is not None:
            img = self.latest_right_bytes
            self.latest_right_bytes = None
            asyncio.run_coroutine_threadsafe(
                self.update_vuer_background(img, key='bg_right', layer=2),
                self.loop,
            )

    async def update_vuer_background(self, img_bytes, key, layer):
        """Update Vuer session background image (stereo: layer 1=left, 2=right)."""
        try:
            if self.current_session is None:
                return
            await self.current_session.upsert(
                ImageBackground(
                    src=img_bytes,
                    key=key,
                    layers=layer,
                    distanceToCamera=2.0,
                    aspect=1.77,
                    height=2.5,
                    position=[0, 0, -2.0],
                    format='jpeg',
                    interpolate=True,
                ),
                to='bgChildren',
            )
        except Exception:
            pass

    def is_valid_float(self, value):
        """Check if value is valid float (excluding NaN, inf)."""
        return isinstance(value, (int, float)) and np.isfinite(value)

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

    # Body head-relative frame from head_inverse @ world:
    # +Y=forward, +Z=right, +X=down. Convert to ROS (+X forward, +Y left, +Z up).
    BODY_HEAD_TO_ROS_POSITION = np.array([
        [0, 1, 0],    # ROS X = head +Y (forward)
        [0, 0, -1],   # ROS Y = -head Z (left)
        [-1, 0, 0]    # ROS Z = -head X (up)
    ])

    def vr_to_ros_transform(self, vr_pos, vr_quat):
        """Transform from VR coordinate system to ROS coordinate system."""
        ros_pos = self.vr_to_ros_matrix @ vr_pos

        vr_rotation = R.from_quat([vr_quat[0], vr_quat[1], vr_quat[2], vr_quat[3]])
        vr_rot_matrix = vr_rotation.as_matrix()
        ros_rot_matrix = self.vr_to_ros_matrix @ vr_rot_matrix
        ros_rotation = R.from_matrix(ros_rot_matrix)
        ros_quat = ros_rotation.as_quat()

        return ros_pos, ros_quat

    def yaw_from_orientation_horizontal(self, ros_quat, fallback_yaw=0.0):
        """
        Compute yaw from head orientation by projecting forward direction.

        Projects forward direction onto horizontal (X-Y) plane.
        Avoids gimbal lock: pitch/roll (tilt) do not affect the returned yaw.
        Forward in head frame is -Z; in ROS frame we use only the X-Y part of
        that direction.
        """
        r = R.from_quat(ros_quat)
        # Head forward = -Z in head frame -> in world (ROS) frame
        forward = r.apply(np.array([0.0, 0.0, -1.0]))
        fx, fy = float(forward[0]), float(forward[1])
        norm_xy = math.sqrt(fx * fx + fy * fy)
        if norm_xy < 1e-6:
            # Looking straight up/down: no horizontal component, use fallback
            return fallback_yaw
        return math.atan2(fy, fx)

    def transform_and_publish_pose(self, pose_array_msg, publisher, hand_name, vr_scale=1.0):
        """Transform wrist pose from head relative coordinates to base_link and publish."""
        if not pose_array_msg.poses:
            return

        wrist_pose_relative = pose_array_msg.poses[0]
        camera_relative_position = np.array([
            wrist_pose_relative.position.x,
            wrist_pose_relative.position.y,
            wrist_pose_relative.position.z
        ], dtype=np.float64)
        camera_relative_quaternion = np.array([
            wrist_pose_relative.orientation.x,
            wrist_pose_relative.orientation.y,
            wrist_pose_relative.orientation.z,
            wrist_pose_relative.orientation.w
        ], dtype=np.float64)

        apply_right_z_flip = (hand_name == 'right')
        self.publish_relative_pose(
            camera_relative_position,
            camera_relative_quaternion,
            publisher,
            vr_scale=vr_scale,
            x_offset=self.wrist_offsets['x'],
            y_offset=self.wrist_offsets['y'],
            z_offset=self.wrist_offsets['z'],
            apply_right_z_flip=apply_right_z_flip,
            pose_role='wrist',
            side=hand_name,
        )

    def publish_relative_pose(
        self,
        camera_relative_position,
        camera_relative_quaternion,
        publisher,
        vr_scale=1.0,
        x_offset=0.0,
        y_offset=0.0,
        z_offset=0.0,
        apply_right_z_flip=False,
        pose_role='wrist',
        side='',
    ):
        """
        Publish a pose in base_link.

        When hand_pose_is_head_relative, rotate position and orientation by
        head pose so head-relative coordinates are correctly expressed in
        base_link.
        """
        # Position: head-relative in VR world axes, then vr_to_ros;
        # no head rotation (axes match base_link).
        scaled_pos = camera_relative_position * vr_scale
        base_position = scaled_pos - self.zedm_to_base_offset

        # When the user squats/stands, shift arm target Z by current head height delta.
        if self.apply_head_height_to_arm_z and pose_role in ('wrist', 'elbow'):
            base_position = base_position.copy()
            base_position[2] += float(self.head_height_offset_for_arms)

        before_zcal = base_position[2]
        if self.zero_z_on_start:
            if (not self.z_calibrated) and pose_role == 'wrist':
                self.z_calibration_offset = base_position[2]
                self.z_calibrated = True
            if self.z_calibrated:
                base_position = base_position.copy()
                base_position[2] -= self.z_calibration_offset

        # Orientation: keep camera-relative; do not multiply by head so pose
        # does not rotate when user rotates body
        camera_relative_rotation = R.from_quat(camera_relative_quaternion)
        if apply_right_z_flip:
            rot_z_180 = R.from_euler('z', 180, degrees=True)
            camera_relative_rotation = camera_relative_rotation * rot_z_180
        arm_quaternion = camera_relative_rotation.as_quat()

        pose_key = f'{side}_{pose_role}' if side else pose_role
        base_position, arm_quaternion = self.low_pass_filter_pose(
            pose_key,
            base_position,
            arm_quaternion,
            max_angle_deg=self.max_wrist_angle_step_deg if pose_role == 'wrist' else None,
        )
        if side:
            base_position = self.apply_elbow_wrist_safety(side, pose_role, base_position)

        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = base_position[0] + x_offset
        target_pose.pose.position.y = base_position[1] + y_offset
        target_pose.pose.position.z = base_position[2] + z_offset

        if pose_role == 'wrist' and rclpy.ok():
            self.wrist_debug_log_counter += 1
            if self.wrist_debug_log_counter % self.wrist_debug_log_every_n == 0:
                self.get_logger().info(
                    f'[WRIST {side}] '
                    f'raw=[{camera_relative_position[0]:+.3f}, '
                    f'{camera_relative_position[1]:+.3f}, '
                    f'{camera_relative_position[2]:+.3f}] '
                    f'*scale({vr_scale})=[{scaled_pos[0]:+.3f}, '
                    f'{scaled_pos[1]:+.3f}, {scaled_pos[2]:+.3f}] '
                    f'before_zcal_z={before_zcal:+.3f} '
                    f'z_cal_off={self.z_calibration_offset:+.3f} '
                    f'final=[{target_pose.pose.position.x:+.3f}, '
                    f'{target_pose.pose.position.y:+.3f}, '
                    f'{target_pose.pose.position.z:+.3f}]'
                )
        target_pose.pose.orientation.x = arm_quaternion[0]
        target_pose.pose.orientation.y = arm_quaternion[1]
        target_pose.pose.orientation.z = arm_quaternion[2]
        target_pose.pose.orientation.w = arm_quaternion[3]
        publisher.publish(target_pose)

    def get_joint_matrix(self, hand_data, joint_index):
        """Extract joint transformation matrix from hand data."""
        arr = (
            hand_data if isinstance(hand_data, np.ndarray)
            else np.asarray(hand_data, dtype=np.float64)
        )
        start_idx = joint_index * 16
        end_idx = start_idx + 16
        matrix_data = arr[start_idx:end_idx]
        return matrix_data.reshape(4, 4, order='F')

    def quat_inverse(self, q):
        """Return the inverse of a quaternion."""
        norm = q.x**2 + q.y**2 + q.z**2 + q.w**2
        if norm == 0:
            return Quaternion()
        inv_norm = 1.0 / norm
        msg = Quaternion()
        msg.x = -q.x * inv_norm
        msg.y = -q.y * inv_norm
        msg.z = -q.z * inv_norm
        msg.w = q.w * inv_norm
        return msg

    def quat_multiply(self, q1, q2):
        """Multiply two quaternions."""
        w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        msg = Quaternion()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.w = w
        return msg

    def get_roll_pitch_yaw(self, q1, q2, cmd=''):
        """Calculate roll, pitch, yaw from two quaternions."""
        q_combined = self.quat_multiply(q1, q2)
        w, x, y, z = q_combined.w, q_combined.x, q_combined.y, q_combined.z

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1, 1)
        pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        if cmd == 'r':
            return roll
        elif cmd == 'p':
            return pitch
        elif cmd == 'y':
            return yaw
        else:
            return roll, pitch, yaw

    def wrap_pi(self, angle):
        """Wrap angle to [-pi, pi] range using modulo."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix."""
        x, y, z, w = q[0], q[1], q[2], q[3]
        rot_matrix = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])
        return rot_matrix

    def process_hand_joints(self, hand_data, side='left'):
        """Process VR hand data (retarget) and publish HandJoints + wrist target pose."""
        if hand_data is None or len(hand_data) != 400:
            return

        hand_joints = HandJoints()
        hand_joints.header.stamp = self.get_clock().now().to_msg()
        hand_joints.header.frame_id = ''
        hand_joints.joints = []
        temp_joints = np.zeros((21, 3), dtype=np.float64)
        pose_counter = 0
        wrist_rot = np.eye(3, dtype=np.float64)
        wrist_pos_ros = None
        wrist_quat_ros = None

        hand_arr = (
            hand_data if isinstance(hand_data, np.ndarray)
            else np.asarray(hand_data, dtype=np.float64)
        )
        head_rot_inv = (
            self.head_inverse_matrix[:3, :3]
            if self.hand_pose_is_head_relative else None
        )
        head_world_pos = (
            self.head_transform_matrix[:3, 3]
            if self.hand_pose_is_head_relative else None
        )

        try:
            for i in self.required_vr_frames:
                start = i * 16
                world_joint_matrix = hand_arr[start:start + 16].reshape(4, 4, order='F')
                world_rot = world_joint_matrix[:3, :3]
                world_pos = world_joint_matrix[:3, 3]

                if (self.hand_pose_is_head_relative and head_rot_inv is not None
                        and head_world_pos is not None):
                    relative_pos_vr = head_rot_inv @ (world_pos - head_world_pos)
                    temp_joints[pose_counter, :] = relative_pos_vr
                    if i == 0:
                        wrist_rot = head_rot_inv @ world_rot
                        wrist_pos_ros = (
                            self.BODY_HEAD_TO_ROS_POSITION @ relative_pos_vr
                        ).astype(np.float64)
                        wrist_quat_ros = (
                            self.body_head_to_ros_rot * R.from_matrix(wrist_rot)
                        ).as_quat()
                else:
                    relative_pos_vr = world_pos
                    temp_joints[pose_counter, :] = relative_pos_vr
                    if i == 0:
                        wrist_rot = world_rot
                        wrist_pos_ros = self.vr_to_ros_matrix @ relative_pos_vr
                        wrist_quat_ros = R.from_matrix(self.vr_to_ros_matrix @ wrist_rot).as_quat()

                pose_counter += 1
        except Exception as e:
            self.get_logger().warn(f'Error processing hand joints for {side}: {e}')
            return

        if pose_counter != 21 or wrist_pos_ros is None or wrist_quat_ros is None:
            return

        if side == 'left':
            if self.start_poses_left:
                temp_joints = (
                    self.low_pass_filter_alpha * temp_joints
                    + (1.0 - self.low_pass_filter_alpha) * self.prev_poses_left
                )
            self.prev_poses_left[:] = temp_joints
            self.start_poses_left = True
            wrist_publisher = self.left_wrist_rviz_pub
            hand_publisher = self.left_hand_pos_pub
        elif side == 'right':
            if self.start_poses_right:
                temp_joints = (
                    self.low_pass_filter_alpha * temp_joints
                    + (1.0 - self.low_pass_filter_alpha) * self.prev_poses_right
                )
            self.prev_poses_right[:] = temp_joints
            self.start_poses_right = True
            wrist_publisher = self.right_wrist_rviz_pub
            hand_publisher = self.right_hand_pos_pub
        else:
            return

        rel_points = temp_joints - temp_joints[0]
        retarget_points = (self.vr_hand_to_urdf @ (wrist_rot.T @ rel_points.T)).T
        for p in retarget_points:
            msg_p = Point32()
            msg_p.x = float(p[0])
            msg_p.y = float(p[1])
            msg_p.z = float(p[2])
            hand_joints.joints.append(msg_p)

        self.publish_relative_pose(
            wrist_pos_ros,
            wrist_quat_ros,
            wrist_publisher,
            vr_scale=self.wrist_vr_scale,
            x_offset=self.wrist_offsets['x'],
            y_offset=self.wrist_offsets['y'],
            z_offset=self.wrist_offsets['z'],
            apply_right_z_flip=(side == 'right'),
            pose_role='wrist',
            side=side,
        )
        hand_publisher.publish(hand_joints)

    def start_vuer_server(self):
        """Start the VR server in a separate thread."""
        def run_server():
            try:
                # Set event loop for this thread
                asyncio.set_event_loop(self.loop)
                self.get_logger().info('Starting VR server...')
                # Register handler and start server
                # spawn(start=True) calls Vuer.start() -> Server.start()
                # Server.start() internally calls run_until_complete + run_forever (blocks here)
                self.vuer.spawn(start=True)(self.main_hand_tracking)
            except Exception as e:
                self.get_logger().error(f'Error in VR server thread: {e}')
                self.get_logger().error(traceback.format_exc())

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    async def main_hand_tracking(self, session):
        """Run main hand tracking session."""
        try:
            fps = self.fps
            self.current_session = session
            self.get_logger().info('Starting hand tracking session')

            # Build bgChildren: optional stereo ImageBackgrounds + Hands
            bg_children = []
            if self.enable_vr_image:
                bg_children.extend([
                    ImageBackground(
                        src='',
                        key='bg_left',
                        layers=1,
                        distanceToCamera=2.0,
                        aspect=1.77,
                        height=2.5,
                        position=[0, 0, -2.0],
                        format='jpeg',
                    ),
                    ImageBackground(
                        src='',
                        key='bg_right',
                        layers=2,
                        distanceToCamera=2.0,
                        aspect=1.77,
                        height=2.5,
                        position=[0, 0, -2.0],
                        format='jpeg',
                    ),
                ])
            bg_children.append(
                Hands(
                    fps=fps,
                    stream=True,
                    key='hands',
                    hideLeft=False,
                    hideRight=False,
                )
            )
            session.upsert(bg_children, to='bgChildren')

            session.upsert(
                Body(
                    key='body_tracking',
                    stream=True,
                    fps=fps,
                    leftHand=False,
                    rightHand=False,
                    hideIndicate=False,
                    showFrame=True,
                    showBody=True,
                    frameScale=0.02,
                ),
                to='children',
            )
            self.get_logger().info(
                f'Hand tracking enabled{" + VR image" if self.enable_vr_image else ""}'
            )
            while True:
                await asyncio.sleep(1/fps)
        except Exception as e:
            self.get_logger().error(f'Error in hand tracking session: {e}')

    async def on_hand_move(self, event, session):
        """Handle hand movement events."""
        try:
            if not self.vr_publishing_enabled:
                return
            if not isinstance(event.value, dict):
                return

            # Only store hand data; processing happens in on_body_tracking_move
            # with same-frame head
            if 'left' in event.value:
                left_data = event.value['left']
                if isinstance(left_data, (list, np.ndarray)) and len(left_data) == 400:
                    self.left_hand_data = (
                        left_data if isinstance(left_data, np.ndarray)
                        else np.asarray(left_data, dtype=np.float64)
                    )
            if 'right' in event.value:
                right_data = event.value['right']
                if isinstance(right_data, (list, np.ndarray)) and len(right_data) == 400:
                    self.right_hand_data = (
                        right_data if isinstance(right_data, np.ndarray)
                        else np.asarray(right_data, dtype=np.float64)
                    )

        except Exception as e:
            self.get_logger().error(f'Error in hand move event: {e}')

    async def on_body_tracking_move(self, event, session):
        """
        Handle body tracking events (head, lift, base, elbow poses).

        Uses 'head' joint for everything: head_transform_matrix, head_inverse_matrix,
        lift (height), head joint, base position XY and yaw.
        Matrix validation rejects degenerate data (zeros from uninitialized tracking).
        """
        try:
            if not rclpy.ok():
                return
            if not self.vr_publishing_enabled:
                return

            if not isinstance(event.value, dict) or not event.value:
                return

            body_data = (
                event.value.get('body')
                if isinstance(event.value, dict) else None
            )
            if not isinstance(body_data, (list, tuple, np.ndarray)):
                return
            body_array = (
                body_data if isinstance(body_data, np.ndarray)
                else np.asarray(body_data, dtype=np.float64)
            )

            # --- Head joint: for all head-related processing ---
            # get_body_joint_matrix_from_flat now rejects degenerate matrices (det~0)
            head_matrix = self.get_body_joint_matrix_from_flat(body_array, 'head')
            if head_matrix is not None:
                self.head_transform_matrix = head_matrix
                self.head_inverse_matrix = np.linalg.inv(head_matrix)

                pos, quat = self.matrix_to_pose(head_matrix)
                ros_pos, ros_quat = self.vr_to_ros_transform(pos, quat)
                self.head_ros_quat = np.array(ros_quat, dtype=np.float64)

                if (rclpy.ok() and hasattr(self, 'wrist_debug_log_counter')
                        and self.wrist_debug_log_counter
                        % self.wrist_debug_log_every_n == 0):
                    self.get_logger().info(
                        f'[HEAD] ros=[{ros_pos[0]:+.3f}, {ros_pos[1]:+.3f}, {ros_pos[2]:+.3f}]'
                    )

                current_camera_height = float(ros_pos[2])
                current_camera_position = np.array(
                    [ros_pos[0], ros_pos[1]], dtype=np.float64
                )
                # Yaw from horizontal projection of head forward to avoid gimbal
                # lock (pitch/roll not affecting yaw)
                fallback_yaw = (
                    float(self.initial_camera_yaw)
                    if self.initial_camera_yaw is not None else 0.0
                )
                current_camera_yaw = self.yaw_from_orientation_horizontal(
                    ros_quat, fallback_yaw=fallback_yaw
                )
                r = R.from_quat(ros_quat)

                # Set initial references (only from validated data)
                if self.initial_camera_height is None:
                    self.initial_camera_height = current_camera_height
                    self.initial_camera_position = current_camera_position.copy()
                    self.initial_camera_yaw = current_camera_yaw
                    self.previous_camera_position = current_camera_position.copy()
                    self.previous_camera_yaw = current_camera_yaw
                if self.initial_odom_position is None and self.current_odom is not None:
                    odom_pos = self.current_odom.pose.pose.position
                    odom_quat = self.current_odom.pose.pose.orientation
                    r_odom = R.from_quat([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])
                    _, _, yaw = r_odom.as_euler('xyz')
                    self.initial_odom_position = np.array([odom_pos.x, odom_pos.y])
                    self.initial_odom_yaw = yaw

                relative_height = current_camera_height - self.initial_camera_height
                # VR device change from initial pose (world-like frame after vr_to_ros_transform)
                relative_position = current_camera_position - self.initial_camera_position
                relative_yaw = self.wrap_pi(current_camera_yaw - self.initial_camera_yaw)

                # Use same-frame head height change for arm target Z (camera-relative behavior).
                self.head_height_offset_for_arms = float(relative_height)

                # Process hands with this head so camera-relative pose uses same-frame head.
                if self.left_hand_data is not None:
                    self.process_hand_joints(self.left_hand_data, 'left')
                if self.right_hand_data is not None:
                    self.process_hand_joints(self.right_hand_data, 'right')

                # Lift: low-pass filter, rate-limit by max_lift_velocity, then publish
                if self.enable_lift_publishing:
                    now = self.get_clock().now()
                    if self.filtered_lift_position is None:
                        self.filtered_lift_position = float(relative_height)
                        self.last_lift_time = now
                    else:
                        target = (
                            self.lift_low_pass_alpha * relative_height
                            + (1.0 - self.lift_low_pass_alpha) * self.filtered_lift_position
                        )
                        if self.max_lift_velocity > 0.0 and self.last_lift_time is not None:
                            dt = (now.nanoseconds - self.last_lift_time.nanoseconds) / 1e9
                            dt = max(0.005, min(0.2, dt))
                            max_step = self.max_lift_velocity * dt
                            delta = target - self.filtered_lift_position
                            if abs(delta) > max_step:
                                self.filtered_lift_position += math.copysign(max_step, delta)
                            else:
                                self.filtered_lift_position = target
                        else:
                            self.filtered_lift_position = target
                        self.last_lift_time = now
                    lift_msg = JointTrajectory()
                    lift_msg.header.stamp.sec = 0
                    lift_msg.header.stamp.nanosec = 0
                    lift_msg.header.frame_id = ''
                    lift_msg.joint_names = ['lift_joint']
                    point = JointTrajectoryPoint()
                    point.positions = [float(self.filtered_lift_position)]
                    point.velocities = [0.0]
                    point.accelerations = [0.0]
                    point.effort = []
                    point.time_from_start.sec = 0
                    point.time_from_start.nanosec = 0
                    lift_msg.points.append(point)
                    if rclpy.ok():
                        self.lift_joint_pub.publish(lift_msg)

                # Base: cmd_vel from head position/yaw error vs odom (P control)
                self.cmd_vel_log_counter += 1
                if (self.initial_camera_position is not None
                        and self.initial_camera_yaw is not None
                        and self.initial_odom_position is not None
                        and self.initial_odom_yaw is not None
                        and self.enable_base_publishing
                        and self.current_odom is not None):
                    current_odom_pos = self.current_odom.pose.pose.position
                    current_odom_quat = self.current_odom.pose.pose.orientation
                    r_odom = R.from_quat([
                        current_odom_quat.x, current_odom_quat.y,
                        current_odom_quat.z, current_odom_quat.w
                    ])
                    _, _, current_odom_yaw = r_odom.as_euler('xyz')
                    current_odom_position = np.array([current_odom_pos.x, current_odom_pos.y])
                    robot_movement_position = current_odom_position - self.initial_odom_position
                    robot_movement_yaw = self.wrap_pi(current_odom_yaw - self.initial_odom_yaw)

                    # position_error is in odom/world frame; rotate to base_link frame
                    # before mapping to cmd_vel linear x/y.
                    position_error = relative_position - robot_movement_position
                    cos_yaw = math.cos(current_odom_yaw)
                    sin_yaw = math.sin(current_odom_yaw)
                    position_error_base = np.array([
                        cos_yaw * position_error[0] + sin_yaw * position_error[1],
                        -sin_yaw * position_error[0] + cos_yaw * position_error[1],
                    ])
                    yaw_error = self.wrap_pi(relative_yaw - robot_movement_yaw)

                    def vel_from_error(err, kp, deadzone, max_vel):
                        if abs(err) <= deadzone:
                            return 0.0
                        vel = kp * err
                        return max(-max_vel, min(max_vel, vel))

                    linear_x = vel_from_error(
                        position_error_base[0], self.base_linear_kp,
                        self.base_linear_deadzone, self.base_max_linear_velocity
                    )
                    linear_y = vel_from_error(
                        position_error_base[1], self.base_linear_kp,
                        self.base_linear_deadzone, self.base_max_linear_velocity
                    )
                    angular_z = vel_from_error(
                        yaw_error, self.base_angular_kp,
                        self.base_angular_deadzone, self.base_max_angular_velocity
                    )

                    twist_msg = Twist()
                    twist_msg.linear.x = linear_x
                    twist_msg.linear.y = linear_y
                    twist_msg.linear.z = 0.0
                    twist_msg.angular.x = 0.0
                    twist_msg.angular.y = 0.0
                    twist_msg.angular.z = angular_z
                    if rclpy.ok():
                        self.cmd_vel_pub.publish(twist_msg)

                    # Divergence detection: error exceeds threshold (e.g. odom jump, VR glitch)
                    position_error_norm = float(np.linalg.norm(position_error))
                    is_diverged = (
                        position_error_norm > self.base_divergence_position_threshold
                        or abs(yaw_error) > self.base_divergence_yaw_threshold
                    )
                    if is_diverged and rclpy.ok():
                        if self.enable_base_debug_topics:
                            self.base_divergence_pub.publish(Bool(data=True))
                        self.base_divergence_log_counter += 1
                        if (self.base_divergence_log_counter
                                % self.base_divergence_log_every_n == 0):
                            self.get_logger().warn(
                                f'[BODY] Base divergence: '
                                f'pos_err_norm={position_error_norm:.3f} '
                                f'(>{self.base_divergence_position_threshold}), '
                                f'yaw_err={yaw_error:+.3f} '
                                f'(|>{self.base_divergence_yaw_threshold})'
                            )
                    else:
                        if self.enable_base_debug_topics:
                            self.base_divergence_pub.publish(Bool(data=False))

                    # Debug topics (enable_base_debug_topics): VR goal pose and
                    # divergence for PlotJuggler vs /odom
                    if self.enable_base_debug_topics and rclpy.ok():
                        vr_goal_position = self.initial_odom_position + relative_position
                        vr_goal_yaw = self.initial_odom_yaw + relative_yaw
                        q_goal = R.from_euler('xyz', [0.0, 0.0, vr_goal_yaw]).as_quat()
                        goal_msg = PoseStamped()
                        goal_msg.header.stamp = self.get_clock().now().to_msg()
                        goal_msg.header.frame_id = 'odom'
                        goal_msg.pose.position.x = float(vr_goal_position[0])
                        goal_msg.pose.position.y = float(vr_goal_position[1])
                        goal_msg.pose.position.z = 0.0
                        goal_msg.pose.orientation.x = float(q_goal[0])
                        goal_msg.pose.orientation.y = float(q_goal[1])
                        goal_msg.pose.orientation.z = float(q_goal[2])
                        goal_msg.pose.orientation.w = float(q_goal[3])
                        self.vr_camera_goal_pub.publish(goal_msg)

                    if (rclpy.ok()
                            and self.cmd_vel_log_counter
                            % self.cmd_vel_log_every_n == 0):
                        self.get_logger().info(
                            f'[BODY] cmd_vel: linear=[{twist_msg.linear.x:+.3f}, '
                            f'{twist_msg.linear.y:+.3f}], '
                            f'angular.z={twist_msg.angular.z:+.3f} | '
                            f'pos_err_world=[{position_error[0]:+.4f}, '
                            f'{position_error[1]:+.4f}], '
                            f'pos_err_base=[{position_error_base[0]:+.4f}, '
                            f'{position_error_base[1]:+.4f}], '
                            f'yaw_err={yaw_error:+.4f}'
                        )

                # Head joint trajectory
                if self.enable_head_publishing:
                    ros_roll, ros_pitch, ros_yaw_head = r.as_euler('xyz')
                    if self.is_valid_float(ros_pitch) and self.is_valid_float(ros_yaw_head):
                        msg = JointTrajectory()
                        msg.joint_names = ['head_joint1', 'head_joint2']
                        point = JointTrajectoryPoint()
                        adjusted_pitch = ros_pitch + self.pitch_offset
                        point.positions = [float(adjusted_pitch), float(ros_yaw_head)]
                        point.velocities = [0.0, 0.0]
                        point.accelerations = [0.0, 0.0]
                        point.effort = []
                        msg.points.append(point)
                        if rclpy.ok():
                            self.head_joint_pub.publish(msg)

            # --- Elbow poses: publish only when current-frame head is valid ---
            # Prevent using stale head_inverse_matrix, which can look world-fixed.
            if head_matrix is not None:
                left_matrix = self.get_body_joint_matrix_from_flat(body_array, 'left-arm-lower')
                right_matrix = self.get_body_joint_matrix_from_flat(body_array, 'right-arm-lower')

                if left_matrix is not None:
                    self.publish_body_joint_pose(left_matrix, self.left_elbow_pub, side='left')
                if right_matrix is not None:
                    self.publish_body_joint_pose(right_matrix, self.right_elbow_pub, side='right')

        except Exception as e:
            if (not rclpy.ok()
                    or 'Destroyable' in str(type(e).__name__)
                    or 'destruction' in str(e).lower()):
                return
            self.get_logger().error(f'Error in body tracking event: {e}')

    def get_body_joint_matrix_from_flat(self, body_array, joint_name):
        """
        Extract a 4x4 matrix for a body joint from flattened body array.

        Returns None if joint not found, data too short, or matrix is degenerate
        (e.g. all zeros when body tracking is not yet initialized).
        """
        if joint_name not in BODY_JOINT_KEYS:
            return None
        index = BODY_JOINT_KEYS.index(joint_name)
        start = index * 16
        end = start + 16
        arr = (
            body_array if isinstance(body_array, np.ndarray)
            else np.asarray(body_array, dtype=np.float64)
        )
        if arr.size < end:
            return None
        matrix = arr[start:end]
        mat4 = matrix.reshape(4, 4, order='F')
        # Reject degenerate matrices (zero matrix, singular, etc.)
        det = np.linalg.det(mat4[:3, :3])
        if abs(det) < 1e-6:
            return None
        return mat4

    def publish_body_joint_pose(self, joint_matrix, publisher, side=''):
        """Publish PoseStamped for a body joint (elbow)."""
        relative_joint_matrix = self.head_inverse_matrix @ joint_matrix
        pos_head, quat_head = self.matrix_to_pose(relative_joint_matrix)
        if not (np.all(np.isfinite(pos_head)) and np.all(np.isfinite(quat_head))):
            return

        # Keep elbow frame conversion identical to wrist head-relative conversion.
        pos_ros = (self.BODY_HEAD_TO_ROS_POSITION @ pos_head).astype(np.float64)
        rel_rot = R.from_matrix(relative_joint_matrix[:3, :3])
        ros_rot = R.from_matrix(self.BODY_HEAD_TO_ROS_POSITION) * rel_rot
        quat_ros = ros_rot.as_quat()
        self.publish_relative_pose(
            pos_ros,
            quat_ros,
            publisher,
            vr_scale=self.elbow_vr_scale,
            x_offset=self.elbow_offsets['x'],
            y_offset=self.elbow_offsets['y'],
            z_offset=self.elbow_offsets['z'],
            apply_right_z_flip=False,
            pose_role='elbow',
            side=side,
        )

    def low_pass_filter_pose(self, key, position, quaternion, max_angle_deg=None):
        """Apply low-pass filter to position and quaternion."""
        quat = np.array(quaternion, dtype=np.float64)
        quat_norm = np.linalg.norm(quat)
        if not np.isfinite(quat_norm) or quat_norm <= 0.0:
            quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        else:
            quat = quat / quat_norm
        if key not in self.pose_filters:
            self.pose_filters[key] = {
                'pos': np.array(position, dtype=np.float64),
                'quat': quat,
            }
            return position, quat

        prev = self.pose_filters[key]
        alpha = 0.5
        filtered_pos = alpha * position + (1.0 - alpha) * prev['pos']
        prev_quat = prev['quat']
        if np.dot(prev_quat, quat) < 0.0:
            quat = -quat
        filtered_quat = self.slerp_quaternion(prev_quat, quat, alpha)
        if max_angle_deg is not None:
            filtered_quat = self.limit_quaternion_spike(prev_quat, filtered_quat, max_angle_deg)

        self.pose_filters[key]['pos'] = filtered_pos
        self.pose_filters[key]['quat'] = filtered_quat
        return filtered_pos, filtered_quat

    def limit_quaternion_spike(self, prev_quat, current_quat, max_angle_deg):
        """Clamp quaternion step by max angle in degrees."""
        prev_quat = np.array(prev_quat, dtype=np.float64)
        curr_quat = np.array(current_quat, dtype=np.float64)
        if np.dot(prev_quat, curr_quat) < 0.0:
            curr_quat = -curr_quat
        dot = float(np.clip(np.dot(prev_quat, curr_quat), -1.0, 1.0))
        angle = 2.0 * math.acos(dot)
        max_angle = math.radians(max_angle_deg)
        if angle <= max_angle or angle <= 1.0e-6:
            return curr_quat
        t = max_angle / angle
        return self.slerp_quaternion(prev_quat, curr_quat, t)

    def slerp_quaternion(self, q0, q1, t):
        """Spherical linear interpolation for quaternions."""
        q0 = np.array(q0, dtype=np.float64)
        q1 = np.array(q1, dtype=np.float64)
        dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        if dot > 0.9995:
            result = q0 + t * (q1 - q0)
            norm = np.linalg.norm(result)
            return result / norm if norm > 0.0 else q0
        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return s0 * q0 + s1 * q1

    def apply_elbow_wrist_safety(self, side, pose_role, position):
        """Clamp wrist-elbow distance to safety limit (wrist has priority)."""
        if pose_role not in ('wrist', 'elbow'):
            return position
        if pose_role == 'wrist':
            return position
        other_key = f'{side}_wrist'
        if other_key not in self.pose_filters:
            return position
        other_pos = self.pose_filters[other_key]['pos']
        delta = position - other_pos
        dist = np.linalg.norm(delta)
        if dist > self.max_elbow_wrist_distance and dist > 0.0:
            position = other_pos + delta * (self.max_elbow_wrist_distance / dist)
        return position

    def __del__(self):
        try:
            if hasattr(self, 'vuer') and hasattr(self.vuer, 'stop'):
                self.vuer.stop()
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
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
