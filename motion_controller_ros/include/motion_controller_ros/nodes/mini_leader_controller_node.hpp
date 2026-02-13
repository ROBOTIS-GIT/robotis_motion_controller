#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <memory>
#include <map>
#include <unordered_map>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "motion_controller_core/kinematics/kinematics_solver.hpp"

using namespace Eigen;

namespace motion_controller_ros
{
class MiniLeaderController : public rclcpp::Node
{
public:
    MiniLeaderController();
    ~MiniLeaderController();

private:
    // Parameters
    double control_frequency_;
    std::string urdf_path_;
    std::string srdf_path_;
    std::string joint_states_topic_;
    std::string right_traj_topic_;
    std::string left_traj_topic_;
    std::string r_goal_pose_topic_;
    std::string l_goal_pose_topic_;
    std::string r_elbow_pose_topic_;
    std::string l_elbow_pose_topic_;
    std::string base_frame_id_;
    std::string base_frame_link_name_;
    std::string r_gripper_name_;
    std::string l_gripper_name_;
    std::string r_elbow_name_;
    std::string l_elbow_name_;
    std::string lift_joint_name_;
    std::string model_prismatic_joint_name_;
    double lift_joint_offset_;

    // Subscribers
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr r_traj_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr l_traj_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r_goal_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr l_goal_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r_elbow_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr l_elbow_pose_pub_;
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Kinematics
    std::shared_ptr<motion_controller::kinematics::KinematicsSolver> kinematics_solver_;

    // State
    VectorXd q_;
    VectorXd qdot_;
    bool right_traj_received_;
    bool left_traj_received_;
    bool lift_joint_received_;
    std::unordered_map<std::string, int> model_joint_index_map_;
    int prismatic_joint_index_;

    // Callbacks
    void rightTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void leftTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void controlLoopCallback();

    // Helpers
    void initializeJointConfig();
    void updateJointPositionsFromTrajectory(const trajectory_msgs::msg::JointTrajectory& msg);
    void updatePrismaticFromJointState(const sensor_msgs::msg::JointState& msg);
    geometry_msgs::msg::PoseStamped makePoseStamped(const Affine3d& pose) const;
    Affine3d computePoseInBaseFrame(const Affine3d& link_pose) const;
};
}  // namespace motion_controller_ros

