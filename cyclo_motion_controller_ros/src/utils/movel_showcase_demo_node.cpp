// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: OpenAI

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robotis_interfaces/msg/move_l.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "kinematics/kinematics_solver.hpp"

namespace cyclo_motion_controller_ros
{

class MoveLShowcaseDemoNode : public rclcpp::Node
{
public:
  MoveLShowcaseDemoNode()
  : Node("movel_showcase_demo_node"),
    initialized_(false),
    current_step_index_(0),
    right_pose_received_(false),
    left_pose_received_(false)
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    right_controlled_link_ =
      this->declare_parameter<std::string>("right_controlled_link", "end_effector_r_link");
    left_controlled_link_ =
      this->declare_parameter<std::string>("left_controlled_link", "end_effector_l_link");
    right_actual_pose_topic_ =
      this->declare_parameter<std::string>("right_actual_pose_topic", "/r_gripper_pose");
    left_actual_pose_topic_ =
      this->declare_parameter<std::string>("left_actual_pose_topic", "/l_gripper_pose");
    right_goal_topic_ = this->declare_parameter<std::string>("right_goal_topic", "/r_goal_move");
    left_goal_topic_ = this->declare_parameter<std::string>("left_goal_topic", "/l_goal_move");
    left_joint_traj_topic_ = this->declare_parameter<std::string>(
      "left_joint_traj_topic",
      "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory");
    left_gripper_joint_name_ = this->declare_parameter<std::string>(
      "left_gripper_joint_name",
      "gripper_l_joint1");
    marker_topic_ =
      this->declare_parameter<std::string>("marker_topic", "/movel_showcase_demo/markers");
    joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    urdf_path_ = this->declare_parameter<std::string>("urdf_path", defaultUrdfPath());
    srdf_path_ = this->declare_parameter<std::string>("srdf_path", defaultSrdfPath());
    start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 1.0);
    dwell_time_sec_ = this->declare_parameter<double>("dwell_time_sec", 1.0);
    fast_segment_sec_ = this->declare_parameter<double>("fast_segment_sec", 1.2);
    slow_segment_sec_ = this->declare_parameter<double>("slow_segment_sec", 2.2);
    setup_segment_sec_ = this->declare_parameter<double>("setup_segment_sec", 1.5);
    square_size_ = this->declare_parameter<double>("square_size", 0.20);
    x_offset_ = this->declare_parameter<double>("x_offset", 0.2);
    z_offset_ = this->declare_parameter<double>("z_offset", 0.2);
    loop_ = this->declare_parameter<bool>("loop", true);
    enable_right_ = this->declare_parameter<bool>("enable_right", true);
    enable_left_ = this->declare_parameter<bool>("enable_left", true);
    show_comparison_legend_ = this->declare_parameter<bool>("show_comparison_legend", true);
    trail_sample_period_ms_ = this->declare_parameter<int>("trail_sample_period_ms", 50);
    trail_min_distance_ = this->declare_parameter<double>("trail_min_distance", 0.003);
    max_trail_points_ = this->declare_parameter<int>("max_trail_points", 500);
    clear_trail_each_cycle_ = this->declare_parameter<bool>("clear_trail_each_cycle", true);
    show_joint_space_prediction_ =
      this->declare_parameter<bool>("show_joint_space_prediction", true);

    right_goal_pub_ = this->create_publisher<robotis_interfaces::msg::MoveL>(right_goal_topic_, 10);
    left_goal_pub_ = this->create_publisher<robotis_interfaces::msg::MoveL>(left_goal_topic_, 10);
    left_joint_traj_pub_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(left_joint_traj_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_,
      rclcpp::QoS(1).transient_local().reliable());
    right_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      right_actual_pose_topic_,
      10,
      std::bind(&MoveLShowcaseDemoNode::rightPoseCallback, this, std::placeholders::_1));
    left_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      left_actual_pose_topic_,
      10,
      std::bind(&MoveLShowcaseDemoNode::leftPoseCallback, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      10,
      std::bind(&MoveLShowcaseDemoNode::jointStateCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    initializeKinematics();

    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&MoveLShowcaseDemoNode::initializeIfReady, this));

    RCLCPP_INFO(this->get_logger(), "MoveL showcase demo node started");
    RCLCPP_INFO(this->get_logger(), "  - Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Right goal topic: %s", right_goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Left goal topic: %s", left_goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Left joint trajectory topic: %s", left_joint_traj_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Right actual pose topic: %s", right_actual_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Left actual pose topic: %s", left_actual_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Joint states topic: %s", joint_states_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Marker topic: %s", marker_topic_.c_str());
  }

private:
  struct DemoStep
  {
    geometry_msgs::msg::PoseStamped right_pose;
    geometry_msgs::msg::PoseStamped left_pose;
    double duration_sec;
    std::string label;
  };

  void initializeIfReady()
  {
    if (initialized_) {
      return;
    }

    if (enable_right_ && !lookupPose(right_controlled_link_, initial_right_pose_)) {
      return;
    }

    if (enable_left_ && !lookupPose(left_controlled_link_, initial_left_pose_)) {
      return;
    }

    buildDemoSequence();
    buildJointSpacePrediction();
    clearTrails();
    publishMarkers();
    initialized_ = true;
    init_timer_->cancel();
    trail_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(10, trail_sample_period_ms_)),
      std::bind(&MoveLShowcaseDemoNode::sampleActualTrails, this));

    RCLCPP_INFO(this->get_logger(), "MoveL showcase demo initialized");
    scheduleNextStep(secondsToNanoseconds(start_delay_sec_));
  }

  bool lookupPose(const std::string & child_frame, geometry_msgs::msg::PoseStamped & pose_out)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(base_frame_, child_frame, tf2::TimePointZero);
      pose_out.header = tf.header;
      pose_out.pose.position.x = tf.transform.translation.x;
      pose_out.pose.position.y = tf.transform.translation.y;
      pose_out.pose.position.z = tf.transform.translation.z;
      pose_out.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  std::string defaultUrdfPath() const
  {
    try {
      return ament_index_cpp::get_package_share_directory("cyclo_motion_controller_models") +
             "/models/ai_worker/ffw_sg2_follower.urdf";
    } catch (const std::exception &) {
      return "";
    }
  }

  std::string defaultSrdfPath() const
  {
    try {
      return ament_index_cpp::get_package_share_directory("cyclo_motion_controller_models") +
             "/models/ai_worker/ffw_sg2_follower_default.srdf";
    } catch (const std::exception &) {
      return "";
    }
  }

  void initializeKinematics()
  {
    if (!show_joint_space_prediction_) {
      return;
    }

    if (urdf_path_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Joint-space prediction disabled: URDF path is empty.");
      return;
    }

    try {
      kinematics_solver_ =
        std::make_shared<cyclo_motion_controller::kinematics::KinematicsSolver>(urdf_path_, srdf_path_);
      model_joint_names_ = kinematics_solver_->getJointNames();
      const int dof = kinematics_solver_->getDof();
      q_current_.setZero(dof);
      qdot_current_.setZero(dof);
      const auto [q_lb, q_ub] = kinematics_solver_->getJointPositionLimit();
      q_lb_ = q_lb;
      q_ub_ = q_ub;

      for (int i = 0; i < dof && i < static_cast<int>(model_joint_names_.size()); ++i) {
        if (model_joint_names_[i].find("arm_r_joint") != std::string::npos) {
          right_arm_indices_.push_back(i);
          right_arm_joint_names_.push_back(model_joint_names_[i]);
        } else if (model_joint_names_[i].find("arm_l_joint") != std::string::npos) {
          left_arm_indices_.push_back(i);
          left_arm_joint_names_.push_back(model_joint_names_[i]);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint-space prediction disabled: failed to initialize kinematics solver: %s",
        e.what());
      kinematics_solver_.reset();
    }
  }

  void buildDemoSequence()
  {
    steps_.clear();

    const double half_side = square_size_ * 0.5;
    const std::vector<std::pair<double, double>> right_offsets = {
      {-half_side, -half_side},
      {-half_side, half_side},
      {half_side, half_side},
      {half_side, -half_side},
      {-half_side, -half_side},
      {0.0, 0.0},
    };
    const std::vector<double> durations = {
      setup_segment_sec_,
      fast_segment_sec_,
      slow_segment_sec_,
      fast_segment_sec_,
      slow_segment_sec_,
      setup_segment_sec_,
    };
    const std::vector<std::string> labels = {
      "Move to showcase start pose",
      "Edge 1: fast straight segment",
      "Edge 2: slow straight segment",
      "Edge 3: fast straight segment",
      "Edge 4: slow straight segment",
      "Return to home pose",
    };

    for (size_t i = 0; i < right_offsets.size(); ++i) {
      DemoStep step;
      step.right_pose = offsetPose(
        initial_right_pose_, x_offset_, right_offsets[i].first, right_offsets[i].second + z_offset_);
      step.left_pose = offsetPose(
        initial_left_pose_, x_offset_, -right_offsets[i].first, right_offsets[i].second + z_offset_);
      step.duration_sec = durations[i];
      step.label = labels[i];
      steps_.push_back(step);
    }
  }

  geometry_msgs::msg::PoseStamped offsetPose(
    const geometry_msgs::msg::PoseStamped & base_pose,
    double dx,
    double dy,
    double dz) const
  {
    auto pose = base_pose;
    pose.header.frame_id = base_frame_;
    pose.pose.position.x += dx;
    pose.pose.position.y += dy;
    pose.pose.position.z += dz;
    return pose;
  }

  void scheduleNextStep(int64_t delay_ns)
  {
    step_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(delay_ns),
      std::bind(&MoveLShowcaseDemoNode::publishCurrentStep, this));
  }

  void publishCurrentStep()
  {
    step_timer_->cancel();

    if (steps_.empty()) {
      return;
    }

    if (current_step_index_ >= steps_.size()) {
      if (!loop_) {
        RCLCPP_INFO(this->get_logger(), "MoveL showcase demo completed");
        return;
      }

      current_step_index_ = 0;
      if (clear_trail_each_cycle_) {
        clearTrails();
      }
      RCLCPP_INFO(this->get_logger(), "Restarting MoveL showcase demo");
    }

    auto & step = steps_[current_step_index_];
    const auto stamp = this->get_clock()->now();

    if (enable_right_) {
      auto msg = createMoveLMessage(step.right_pose, stamp, step.duration_sec);
      right_goal_pub_->publish(msg);
    }

    if (enable_left_) {
      auto msg = createLeftJointTrajectoryMessage(step.duration_sec, current_step_index_);
      if (!msg.joint_names.empty()) {
        left_joint_traj_pub_->publish(msg);
      }
    }

    publishMarkers();

    RCLCPP_INFO(
      this->get_logger(),
      "Published step %zu/%zu: %s (duration %.2f sec)",
      current_step_index_ + 1,
      steps_.size(),
      step.label.c_str(),
      step.duration_sec);

    const double wait_sec = step.duration_sec + dwell_time_sec_;
    ++current_step_index_;
    scheduleNextStep(secondsToNanoseconds(wait_sec));
  }

  robotis_interfaces::msg::MoveL createMoveLMessage(
    const geometry_msgs::msg::PoseStamped & pose,
    const rclcpp::Time & stamp,
    double duration_sec) const
  {
    robotis_interfaces::msg::MoveL msg;
    msg.pose = pose;
    msg.pose.header.stamp = stamp;
    msg.time_from_start = toDurationMsg(duration_sec);
    return msg;
  }

  trajectory_msgs::msg::JointTrajectory createLeftJointTrajectoryMessage(
    double duration_sec,
    size_t step_index) const
  {
    trajectory_msgs::msg::JointTrajectory msg;

    if (step_index >= predicted_left_target_qs_.size() || left_arm_joint_names_.empty()) {
      return msg;
    }

    msg.joint_names = left_arm_joint_names_;
    msg.joint_names.push_back(left_gripper_joint_name_);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    const Eigen::VectorXd & target_q = predicted_left_target_qs_[step_index];
    for (const int idx : left_arm_indices_) {
      if (idx >= 0 && idx < target_q.size()) {
        point.positions.push_back(target_q[idx]);
      }
    }
    point.positions.push_back(left_gripper_position_);
    point.time_from_start = toDurationMsg(duration_sec);
    msg.points.push_back(point);
    return msg;
  }

  builtin_interfaces::msg::Duration toDurationMsg(double seconds) const
  {
    builtin_interfaces::msg::Duration duration;
    const auto total_ns = secondsToNanoseconds(seconds);
    duration.sec = static_cast<int32_t>(total_ns / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
    return duration;
  }

  int64_t secondsToNanoseconds(double seconds) const
  {
    return static_cast<int64_t>(seconds * 1000000000.0);
  }

  void sampleActualTrails()
  {
    if (!initialized_) {
      return;
    }

    if (enable_right_ && right_pose_received_) {
      appendTrailPoint(right_trail_points_, latest_right_pose_.pose.position);
    } else if (enable_right_) {
      geometry_msgs::msg::PoseStamped pose;
      if (lookupPose(right_controlled_link_, pose)) {
        appendTrailPoint(right_trail_points_, pose.pose.position);
      }
    }

    if (enable_left_) {
      geometry_msgs::msg::PoseStamped pose;
      if (lookupPose(left_controlled_link_, pose)) {
        appendTrailPoint(left_trail_points_, pose.pose.position);
      } else if (left_pose_received_) {
        appendTrailPoint(left_trail_points_, latest_left_pose_.pose.position);
      }
    }

    publishMarkers();
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || !kinematics_solver_) {
      return;
    }

    std::unordered_map<std::string, size_t> msg_joint_index;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      msg_joint_index[msg->name[i]] = i;
    }

    const int max_index = std::min<int>(kinematics_solver_->getDof(), static_cast<int>(model_joint_names_.size()));
    for (int i = 0; i < max_index; ++i) {
      const auto it = msg_joint_index.find(model_joint_names_[i]);
      if (it == msg_joint_index.end()) {
        continue;
      }
      const size_t msg_idx = it->second;
      if (msg_idx < msg->position.size()) {
        q_current_[i] = msg->position[msg_idx];
      }
      if (msg_idx < msg->velocity.size()) {
        qdot_current_[i] = msg->velocity[msg_idx];
      }
    }

    joint_state_received_ = true;
    const auto gripper_it = msg_joint_index.find(left_gripper_joint_name_);
    if (gripper_it != msg_joint_index.end() && gripper_it->second < msg->position.size()) {
      left_gripper_position_ = msg->position[gripper_it->second];
    }
    if (initialized_ && !joint_space_prediction_attempted_) {
      if (buildJointSpacePrediction()) {
        RCLCPP_INFO(this->get_logger(), "Built joint-space prediction path for comparison.");
      }
      publishMarkers();
    }
  }

  void appendTrailPoint(
    std::vector<geometry_msgs::msg::Point> & trail,
    const geometry_msgs::msg::Point & point)
  {
    if (!trail.empty()) {
      const auto & last = trail.back();
      const double dx = point.x - last.x;
      const double dy = point.y - last.y;
      const double dz = point.z - last.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < trail_min_distance_) {
        return;
      }
    }

    trail.push_back(point);
    if (static_cast<int>(trail.size()) > max_trail_points_) {
      trail.erase(trail.begin());
    }
  }

  void clearTrails()
  {
    right_trail_points_.clear();
    left_trail_points_.clear();
  }

  bool buildJointSpacePrediction()
  {
    predicted_right_path_.clear();
    predicted_left_path_.clear();
    predicted_left_target_qs_.clear();

    if (!show_joint_space_prediction_ || !kinematics_solver_ || !joint_state_received_ || steps_.empty()) {
      return false;
    }

    joint_space_prediction_attempted_ = true;

    Eigen::VectorXd right_seed = q_current_;
    Eigen::VectorXd left_seed = q_current_;

    if (enable_right_) {
      if (!buildPredictedPathForArm(
            right_seed,
            steps_,
            right_controlled_link_,
            right_arm_indices_,
            true,
            predicted_right_path_,
            nullptr))
      {
        if (!right_prediction_warned_) {
          RCLCPP_WARN(this->get_logger(), "Failed to build right-arm joint-space prediction path.");
          right_prediction_warned_ = true;
        }
      }
    }

    if (enable_left_) {
      if (!buildPredictedPathForArm(
            left_seed,
            steps_,
            left_controlled_link_,
            left_arm_indices_,
            false,
            predicted_left_path_,
            &predicted_left_target_qs_))
      {
        if (!left_prediction_warned_) {
          RCLCPP_WARN(this->get_logger(), "Failed to build left-arm joint-space prediction path.");
          left_prediction_warned_ = true;
        }
      }
    }

    return !predicted_right_path_.empty() || !predicted_left_path_.empty();
  }

  bool buildPredictedPathForArm(
    Eigen::VectorXd & seed_q,
    const std::vector<DemoStep> & steps,
    const std::string & link_name,
    const std::vector<int> & arm_indices,
    bool is_right,
    std::vector<geometry_msgs::msg::Point> & path_out,
    std::vector<Eigen::VectorXd> * target_qs_out)
  {
    if (!kinematics_solver_ || arm_indices.empty()) {
      return false;
    }

    path_out.clear();
    if (target_qs_out != nullptr) {
      target_qs_out->clear();
    }
    Eigen::VectorXd current_q = seed_q;
    const int interpolation_samples = 25;

    for (const auto & step : steps) {
      const auto & target_pose_msg = is_right ? step.right_pose : step.left_pose;
      Eigen::Affine3d target_pose = poseMsgToEigen(target_pose_msg);
      Eigen::VectorXd target_q = current_q;
      if (!solveArmIk(target_q, target_pose, link_name, arm_indices)) {
        if (target_qs_out != nullptr) {
          target_qs_out->clear();
        }
        return false;
      }

      if (target_qs_out != nullptr) {
        target_qs_out->push_back(target_q);
      }

      for (int sample = 0; sample <= interpolation_samples; ++sample) {
        const double alpha = static_cast<double>(sample) / interpolation_samples;
        Eigen::VectorXd q_interp = current_q;
        for (const int idx : arm_indices) {
          q_interp[idx] = (1.0 - alpha) * current_q[idx] + alpha * target_q[idx];
        }

        const auto pose = kinematics_solver_->computePose(q_interp, link_name);
        appendUniquePoint(path_out, eigenTranslationToPoint(pose.translation()));
      }

      current_q = target_q;
    }

    seed_q = current_q;
    return true;
  }

  bool solveArmIk(
    Eigen::VectorXd & q_io,
    const Eigen::Affine3d & target_pose,
    const std::string & link_name,
    const std::vector<int> & arm_indices)
  {
    if (!kinematics_solver_) {
      return false;
    }

    constexpr int kMaxIters = 400;
    constexpr double kLambda = 1e-4;
    constexpr double kStepScale = 0.5;
    constexpr double kMaxStepNorm = 0.10;
    constexpr double kPosTol = 5e-3;
    constexpr double kOriTol = 0.12;
    constexpr double kRelaxedPosTol = 1.5e-2;
    constexpr double kRelaxedOriTol = 0.35;

    Eigen::VectorXd best_q = q_io;
    double best_pos_error_norm = std::numeric_limits<double>::infinity();
    double best_ori_error_norm = std::numeric_limits<double>::infinity();
    double best_cost = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < kMaxIters; ++iter) {
      const Eigen::Affine3d current_pose = kinematics_solver_->computePose(q_io, link_name);
      const Eigen::Vector3d position_error = target_pose.translation() - current_pose.translation();
      const Eigen::Matrix3d rotation_error =
        target_pose.linear() * current_pose.linear().transpose();
      const Eigen::AngleAxisd angle_axis_error(rotation_error);
      Eigen::Vector3d orientation_error = Eigen::Vector3d::Zero();
      if (std::abs(angle_axis_error.angle()) > 1e-9) {
        orientation_error = angle_axis_error.axis() * angle_axis_error.angle();
      }

      const double pos_error_norm = position_error.norm();
      const double ori_error_norm = orientation_error.norm();
      const double cost = pos_error_norm + 0.35 * ori_error_norm;
      if (cost < best_cost) {
        best_cost = cost;
        best_q = q_io;
        best_pos_error_norm = pos_error_norm;
        best_ori_error_norm = ori_error_norm;
      }

      if (pos_error_norm < kPosTol && ori_error_norm < kOriTol) {
        return true;
      }

      const Eigen::MatrixXd jacobian_full = kinematics_solver_->computeJacobian(q_io, link_name);
      Eigen::MatrixXd jacobian_reduced(6, arm_indices.size());
      for (size_t col = 0; col < arm_indices.size(); ++col) {
        jacobian_reduced.col(col) = jacobian_full.col(arm_indices[col]);
      }

      Eigen::Matrix<double, 6, 1> task_error;
      task_error.head<3>() = position_error;
      task_error.tail<3>() = orientation_error;

      Eigen::Matrix<double, 6, 6> task_weight =
        Eigen::Matrix<double, 6, 6>::Identity();
      task_weight.topLeftCorner<3, 3>() *= 1.0;
      task_weight.bottomRightCorner<3, 3>() *= 0.5;

      const Eigen::Matrix<double, 6, 6> damping =
        jacobian_reduced * jacobian_reduced.transpose() +
        kLambda * Eigen::Matrix<double, 6, 6>::Identity();
      Eigen::VectorXd dq =
        jacobian_reduced.transpose() *
        damping.ldlt().solve(task_weight * task_error * kStepScale);

      const double dq_norm = dq.norm();
      if (dq_norm > kMaxStepNorm) {
        dq *= (kMaxStepNorm / dq_norm);
      }

      for (size_t i = 0; i < arm_indices.size(); ++i) {
        const int idx = arm_indices[i];
        q_io[idx] += dq[i];
        q_io[idx] = std::min(q_ub_[idx], std::max(q_lb_[idx], q_io[idx]));
      }
    }

    if (best_pos_error_norm < kRelaxedPosTol && best_ori_error_norm < kRelaxedOriTol) {
      q_io = best_q;
      return true;
    }

    return false;
  }

  Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::PoseStamped & pose_msg) const
  {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z;
    const Eigen::Quaterniond quat(
      pose_msg.pose.orientation.w,
      pose_msg.pose.orientation.x,
      pose_msg.pose.orientation.y,
      pose_msg.pose.orientation.z);
    pose.linear() = quat.normalized().toRotationMatrix();
    return pose;
  }

  geometry_msgs::msg::Point eigenTranslationToPoint(const Eigen::Vector3d & translation) const
  {
    geometry_msgs::msg::Point point;
    point.x = translation.x();
    point.y = translation.y();
    point.z = translation.z();
    return point;
  }

  void appendUniquePoint(
    std::vector<geometry_msgs::msg::Point> & points,
    const geometry_msgs::msg::Point & point) const
  {
    if (!points.empty()) {
      const auto & last = points.back();
      const double dx = point.x - last.x;
      const double dy = point.y - last.y;
      const double dz = point.z - last.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < 1e-4) {
        return;
      }
    }
    points.push_back(point);
  }

  void rightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_right_pose_ = *msg;
    right_pose_received_ = true;
  }

  void leftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_left_pose_ = *msg;
    left_pose_received_ = true;
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    addDeleteAllMarker(marker_array);

    if (enable_right_) {
      addActualTrailMarker(marker_array, true);
    }

    if (enable_left_) {
      addActualTrailMarker(marker_array, false);
    }

    marker_pub_->publish(marker_array);
  }

  void addDeleteAllMarker(visualization_msgs::msg::MarkerArray & marker_array) const
  {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
  }

  void addPathMarkers(
    visualization_msgs::msg::MarkerArray & marker_array,
    const geometry_msgs::msg::PoseStamped & initial_pose,
    bool is_right) const
  {
    const std::string side = is_right ? "right" : "left";
    const std::vector<geometry_msgs::msg::Point> path_points = buildPathPoints(is_right);

    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = base_frame_;
    path_marker.header.stamp = this->get_clock()->now();
    path_marker.ns = side + "_movel_path";
    path_marker.id = is_right ? 0 : 100;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.01;
    path_marker.color.a = 1.0;
    path_marker.color.r = is_right ? 0.1f : 0.2f;
    path_marker.color.g = is_right ? 0.8f : 0.6f;
    path_marker.color.b = is_right ? 1.0f : 0.2f;
    path_marker.points = path_points;
    marker_array.markers.push_back(path_marker);

    visualization_msgs::msg::Marker waypoint_marker;
    waypoint_marker.header = path_marker.header;
    waypoint_marker.ns = side + "_movel_waypoints";
    waypoint_marker.id = is_right ? 1 : 101;
    waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
    waypoint_marker.scale.x = 0.025;
    waypoint_marker.scale.y = 0.025;
    waypoint_marker.scale.z = 0.025;
    waypoint_marker.color = path_marker.color;
    waypoint_marker.points = path_points;
    marker_array.markers.push_back(waypoint_marker);

    (void)initial_pose;
  }

  std::vector<geometry_msgs::msg::Point> buildPathPoints(bool is_right) const
  {
    std::vector<geometry_msgs::msg::Point> points;
    for (const auto & step : steps_) {
      const auto & pose = is_right ? step.right_pose.pose : step.left_pose.pose;
      geometry_msgs::msg::Point point;
      point.x = pose.position.x;
      point.y = pose.position.y;
      point.z = pose.position.z;
      points.push_back(point);
    }
    return points;
  }

  void addActualTrailMarker(
    visualization_msgs::msg::MarkerArray & marker_array,
    bool is_right) const
  {
    const std::string side = is_right ? "right" : "left";
    const auto & trail = is_right ? right_trail_points_ : left_trail_points_;

    visualization_msgs::msg::Marker trail_marker;
    trail_marker.header.frame_id = base_frame_;
    trail_marker.header.stamp = this->get_clock()->now();
    trail_marker.ns = side + "_actual_trail";
    trail_marker.id = is_right ? 50 : 150;
    trail_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trail_marker.action = visualization_msgs::msg::Marker::ADD;
    trail_marker.scale.x = 0.012;
    trail_marker.color.a = 1.0;
    trail_marker.color.r = is_right ? 1.0f : 1.0f;
    trail_marker.color.g = is_right ? 0.7f : 0.3f;
    trail_marker.color.b = is_right ? 0.1f : 0.8f;
    trail_marker.points = trail;
    marker_array.markers.push_back(trail_marker);
  }

  void addPredictedJointSpaceMarker(
    visualization_msgs::msg::MarkerArray & marker_array,
    bool is_right) const
  {
    const auto & points = is_right ? predicted_right_path_ : predicted_left_path_;
    if (points.empty()) {
      return;
    }

    const std::string side = is_right ? "right" : "left";
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = side + "_joint_space_prediction";
    marker.id = is_right ? 60 : 160;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.008;
    marker.color.a = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.points = points;
    marker_array.markers.push_back(marker);
  }

  void addActiveTargetMarkers(visualization_msgs::msg::MarkerArray & marker_array) const
  {
    if (steps_.empty()) {
      return;
    }

    const size_t active_index = std::min(current_step_index_, steps_.size() - 1);

    if (enable_right_) {
      marker_array.markers.push_back(
        createActiveTargetMarker(steps_[active_index].right_pose.pose.position, "right_active_target", 200));
    }

    if (enable_left_) {
      marker_array.markers.push_back(
        createActiveTargetMarker(steps_[active_index].left_pose.pose.position, "left_active_target", 201));
    }
  }

  visualization_msgs::msg::Marker createActiveTargetMarker(
    const geometry_msgs::msg::Point & point,
    const std::string & ns,
    int id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.045;
    marker.scale.y = 0.045;
    marker.scale.z = 0.045;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    return marker;
  }

  std::string base_frame_;
  std::string right_controlled_link_;
  std::string left_controlled_link_;
  std::string right_actual_pose_topic_;
  std::string left_actual_pose_topic_;
  std::string right_goal_topic_;
  std::string left_goal_topic_;
  std::string left_joint_traj_topic_;
  std::string left_gripper_joint_name_;
  std::string marker_topic_;
  std::string joint_states_topic_;
  std::string urdf_path_;
  std::string srdf_path_;
  double start_delay_sec_;
  double dwell_time_sec_;
  double fast_segment_sec_;
  double slow_segment_sec_;
  double setup_segment_sec_;
  double square_size_;
  double x_offset_;
  double z_offset_;
  bool loop_;
  bool enable_right_;
  bool enable_left_;
  bool show_comparison_legend_;
  int trail_sample_period_ms_;
  double trail_min_distance_;
  int max_trail_points_;
  bool clear_trail_each_cycle_;
  bool show_joint_space_prediction_;
  bool initialized_;
  size_t current_step_index_;
  bool right_pose_received_;
  bool left_pose_received_;
  bool joint_state_received_ = false;
  bool joint_space_prediction_attempted_ = false;
  bool right_prediction_warned_ = false;
  bool left_prediction_warned_ = false;
  geometry_msgs::msg::PoseStamped initial_right_pose_;
  geometry_msgs::msg::PoseStamped initial_left_pose_;
  geometry_msgs::msg::PoseStamped latest_right_pose_;
  geometry_msgs::msg::PoseStamped latest_left_pose_;
  std::vector<geometry_msgs::msg::Point> right_trail_points_;
  std::vector<geometry_msgs::msg::Point> left_trail_points_;
  std::vector<geometry_msgs::msg::Point> predicted_right_path_;
  std::vector<geometry_msgs::msg::Point> predicted_left_path_;
  std::vector<Eigen::VectorXd> predicted_left_target_qs_;
  std::vector<DemoStep> steps_;
  Eigen::VectorXd q_current_;
  Eigen::VectorXd qdot_current_;
  Eigen::VectorXd q_lb_;
  Eigen::VectorXd q_ub_;
  std::vector<std::string> model_joint_names_;
  std::vector<std::string> right_arm_joint_names_;
  std::vector<std::string> left_arm_joint_names_;
  std::vector<int> right_arm_indices_;
  std::vector<int> left_arm_indices_;
  double left_gripper_position_ = 0.0;
  std::shared_ptr<cyclo_motion_controller::kinematics::KinematicsSolver> kinematics_solver_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<robotis_interfaces::msg::MoveL>::SharedPtr right_goal_pub_;
  rclcpp::Publisher<robotis_interfaces::msg::MoveL>::SharedPtr left_goal_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_joint_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr trail_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace cyclo_motion_controller_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cyclo_motion_controller_ros::MoveLShowcaseDemoNode>());
  rclcpp::shutdown();
  return 0;
}
