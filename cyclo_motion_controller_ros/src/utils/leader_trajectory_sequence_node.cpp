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

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace cyclo_motion_controller_ros
{

class LeaderTrajectorySequenceNode : public rclcpp::Node
{
public:
  LeaderTrajectorySequenceNode()
  : Node("leader_trajectory_sequence_node"), current_step_index_(0)
  {
    right_topic_ = this->declare_parameter<std::string>(
      "right_topic",
      "/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory");
    left_topic_ = this->declare_parameter<std::string>(
      "left_topic",
      "/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory");
    initial_delay_sec_ = this->declare_parameter<double>("initial_delay_sec", 1.0);
    step_gap_sec_ = this->declare_parameter<double>("step_gap_sec", 1.0);

    right_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(right_topic_, 10);
    left_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(left_topic_, 10);

    buildSequence();

    RCLCPP_INFO(this->get_logger(), "Leader trajectory sequence node started");
    RCLCPP_INFO(this->get_logger(), "  - Right topic: %s", right_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Left topic: %s", left_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Initial delay: %.2f sec", initial_delay_sec_);
    RCLCPP_INFO(this->get_logger(), "  - Step gap: %.2f sec", step_gap_sec_);

    scheduleNextPublish(secondsToNanoseconds(initial_delay_sec_));
  }

private:
  struct TrajectoryStep
  {
    trajectory_msgs::msg::JointTrajectory right;
    trajectory_msgs::msg::JointTrajectory left;
  };

  void buildSequence()
  {
    steps_.push_back(
      {
        createTrajectory(
          {
            "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
            "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1"
          },
          {-0.5, -0.2, 0.0, -1.9, 0.0, -0.5, 0.0, 0.0},
          3),
        createTrajectory(
          {
            "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
            "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1"
          },
          {-0.5, 0.2, 0.0, -1.9, 0.0, -0.5, 0.0, 0.0},
          3)
      });

    steps_.push_back(
      {
        createTrajectory(
          {
            "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
            "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1"
          },
          {-0.5, -0.2, -0.4, -1.9, 0.0, -0.5, 0.0, -1.0},
          3),
        createTrajectory(
          {
            "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
            "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1"
          },
          {-0.5, 0.2, 0.4, -1.9, 0.0, -0.5, 0.0, 0.0},
          3)
      });
  }

  trajectory_msgs::msg::JointTrajectory createTrajectory(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions,
    int32_t duration_sec) const
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start.sec = duration_sec;
    point.time_from_start.nanosec = 0;
    trajectory.points.push_back(point);

    return trajectory;
  }

  void scheduleNextPublish(int64_t delay_ns)
  {
    publish_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(delay_ns),
      std::bind(&LeaderTrajectorySequenceNode::publishCurrentStep, this));
  }

  void publishCurrentStep()
  {
    publish_timer_->cancel();

    if (current_step_index_ >= steps_.size()) {
      RCLCPP_INFO(this->get_logger(), "Trajectory sequence already completed");
      return;
    }

    auto & step = steps_[current_step_index_];
    const auto stamp = this->now();
    step.right.header.stamp = stamp;
    step.left.header.stamp = stamp;

    // Publish both arm trajectories back-to-back so the sequence step starts together.
    right_pub_->publish(step.right);
    left_pub_->publish(step.left);

    const auto wait_ns =
      std::max(getTrajectoryDurationNanoseconds(step.right), getTrajectoryDurationNanoseconds(step.left)) +
      secondsToNanoseconds(step_gap_sec_);

    RCLCPP_INFO(
      this->get_logger(),
      "Published trajectory step %zu/%zu. Next step in %.2f sec",
      current_step_index_ + 1,
      steps_.size(),
      static_cast<double>(wait_ns) / 1e9);

    ++current_step_index_;

    if (current_step_index_ < steps_.size()) {
      scheduleNextPublish(wait_ns);
    } else {
      RCLCPP_INFO(this->get_logger(), "All trajectory steps published");
    }
  }

  int64_t getTrajectoryDurationNanoseconds(
    const trajectory_msgs::msg::JointTrajectory & trajectory) const
  {
    int64_t max_duration_ns = 0;

    for (const auto & point : trajectory.points) {
      const int64_t point_duration_ns =
        static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
        static_cast<int64_t>(point.time_from_start.nanosec);
      max_duration_ns = std::max(max_duration_ns, point_duration_ns);
    }

    return max_duration_ns;
  }

  int64_t secondsToNanoseconds(double seconds) const
  {
    return static_cast<int64_t>(seconds * 1000000000.0);
  }

  std::string right_topic_;
  std::string left_topic_;
  double initial_delay_sec_;
  double step_gap_sec_;
  size_t current_step_index_;
  std::vector<TrajectoryStep> steps_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace cyclo_motion_controller_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cyclo_motion_controller_ros::LeaderTrajectorySequenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
