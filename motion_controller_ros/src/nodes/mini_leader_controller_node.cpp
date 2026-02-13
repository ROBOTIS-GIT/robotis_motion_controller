#include "motion_controller_ros/nodes/mini_leader_controller_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace motion_controller_ros
{
    MiniLeaderController::MiniLeaderController()
        : Node("mini_leader_controller"),
          right_traj_received_(false),
          left_traj_received_(false),
          lift_joint_received_(false),
          prismatic_joint_index_(-1)
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Mini Leader Controller - Starting up...");
        RCLCPP_INFO(this->get_logger(), "Node name: %s", this->get_name());
        RCLCPP_INFO(this->get_logger(), "========================================");

        control_frequency_ = this->declare_parameter("control_frequency", 100.0);
        urdf_path_ = this->declare_parameter("urdf_path", std::string(""));
        srdf_path_ = this->declare_parameter("srdf_path", std::string(""));
        joint_states_topic_ = this->declare_parameter("joint_states_topic", std::string("/joint_states"));
        right_traj_topic_ = this->declare_parameter(
            "right_traj_topic",
            std::string("/leader/joint_trajectory_command_broadcaster_right/joint_trajectory"));
        left_traj_topic_ = this->declare_parameter(
            "left_traj_topic",
            std::string("/leader/joint_trajectory_command_broadcaster_left/joint_trajectory"));
        r_goal_pose_topic_ = this->declare_parameter("r_goal_pose_topic", std::string("/r_goal_pose"));
        l_goal_pose_topic_ = this->declare_parameter("l_goal_pose_topic", std::string("/l_goal_pose"));
        r_elbow_pose_topic_ = this->declare_parameter("r_elbow_pose_topic", std::string("/r_elbow_pose"));
        l_elbow_pose_topic_ = this->declare_parameter("l_elbow_pose_topic", std::string("/l_elbow_pose"));
        base_frame_id_ = this->declare_parameter("base_frame_id", std::string("base_link"));
        base_frame_link_name_ = this->declare_parameter("base_frame_link_name", std::string("world"));
        r_gripper_name_ = this->declare_parameter("r_gripper_name", std::string("arm_r_link7"));
        l_gripper_name_ = this->declare_parameter("l_gripper_name", std::string("arm_l_link7"));
        r_elbow_name_ = this->declare_parameter("r_elbow_name", std::string("arm_r_link4"));
        l_elbow_name_ = this->declare_parameter("l_elbow_name", std::string("arm_l_link4"));
        lift_joint_name_ = this->declare_parameter("lift_joint_name", std::string("lift_joint"));
        model_prismatic_joint_name_ = this->declare_parameter("model_prismatic_joint_name", std::string("joint"));

        try {
            if (urdf_path_.empty() || srdf_path_.empty()) {
                const std::string package_path =
                    ament_index_cpp::get_package_share_directory("ffw_description");
                if (urdf_path_.empty()) {
                    urdf_path_ = package_path + "/urdf/ffw_lg2_leader/ffw_lg2_leader.urdf";
                }
                if (srdf_path_.empty()) {
                    srdf_path_ = package_path + "/urdf/ffw_lg2_leader/ffw_lg2_leader.srdf";
                }
            }
            RCLCPP_INFO(this->get_logger(), "URDF path: %s", urdf_path_.c_str());
            RCLCPP_INFO(this->get_logger(), "SRDF path: %s", srdf_path_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to resolve URDF/SRDF paths: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        try {
            kinematics_solver_ =
                std::make_shared<motion_controller::kinematics::KinematicsSolver>(urdf_path_, srdf_path_);
            const int dof = kinematics_solver_->getDof();
            q_.setZero(dof);
            qdot_.setZero(dof);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize kinematics: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        initializeJointConfig();

        r_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            right_traj_topic_, 10,
            std::bind(&MiniLeaderController::rightTrajectoryCallback, this, std::placeholders::_1));
        l_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            left_traj_topic_, 10,
            std::bind(&MiniLeaderController::leftTrajectoryCallback, this, std::placeholders::_1));
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_, 10,
            std::bind(&MiniLeaderController::jointStateCallback, this, std::placeholders::_1));

        r_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            r_goal_pose_topic_, 10);
        l_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            l_goal_pose_topic_, 10);
        r_elbow_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            r_elbow_pose_topic_, 10);
        l_elbow_pose_pub_ =this->create_publisher<geometry_msgs::msg::PoseStamped>(
            l_elbow_pose_topic_, 10); 

        const int timer_period_ms = static_cast<int>(1000.0 / control_frequency_);
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&MiniLeaderController::controlLoopCallback, this));
    }

    MiniLeaderController::~MiniLeaderController() = default;

    void MiniLeaderController::initializeJointConfig()
    {
        const auto joint_names = kinematics_solver_->getJointNames();
        model_joint_index_map_.clear();
        for (size_t i = 0; i < joint_names.size(); ++i) {
            model_joint_index_map_[joint_names[i]] = static_cast<int>(i);
        }

        auto it = model_joint_index_map_.find(model_prismatic_joint_name_);
        if (it != model_joint_index_map_.end()) {
            prismatic_joint_index_ = it->second;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Model prismatic joint '%s' not found in URDF.", model_prismatic_joint_name_.c_str());
        }
    }

    void MiniLeaderController::rightTrajectoryCallback(
        const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        updateJointPositionsFromTrajectory(*msg);
        right_traj_received_ = true;
    }

    void MiniLeaderController::leftTrajectoryCallback(
        const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        updateJointPositionsFromTrajectory(*msg);
        left_traj_received_ = true;
    }

    void MiniLeaderController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        updatePrismaticFromJointState(*msg);
    }

    void MiniLeaderController::updateJointPositionsFromTrajectory(
        const trajectory_msgs::msg::JointTrajectory& msg)
    {
        if (msg.points.empty()) {
            return;
        }
        const auto& point = msg.points.front();
        if (point.positions.empty()) {
            return;
        }

        for (size_t i = 0; i < msg.joint_names.size(); ++i) {
            auto it = model_joint_index_map_.find(msg.joint_names[i]);
            if (it == model_joint_index_map_.end()) {
                continue;
            }
            const int model_index = it->second;
            if (model_index < 0 || model_index >= q_.size()) {
                continue;
            }
            if (i < point.positions.size()) {
                q_[model_index] = point.positions[i];
            }
            if (i < point.velocities.size()) {
                qdot_[model_index] = point.velocities[i];
            }
        }
    }

    void MiniLeaderController::updatePrismaticFromJointState(const sensor_msgs::msg::JointState& msg)
    {
        if (prismatic_joint_index_ < 0 || prismatic_joint_index_ >= q_.size()) {
            return;
        }

        for (size_t i = 0; i < msg.name.size(); ++i) {
            if (msg.name[i] != lift_joint_name_) {
                continue;
            }
            if (i < msg.position.size()) {
                q_[prismatic_joint_index_] = msg.position[i];
                lift_joint_received_ = true;
            }
            if (i < msg.velocity.size()) {
                qdot_[prismatic_joint_index_] = msg.velocity[i];
            }
            return;
        }
    }

    void MiniLeaderController::controlLoopCallback()
    {
        if (!lift_joint_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for lift_joint from /joint_states...");
            return;
        }
        if (!right_traj_received_ && !left_traj_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for joint trajectory commands...");
            return;
        }

        try {
            kinematics_solver_->updateState(q_, qdot_);
            const Affine3d r_pose =
                computePoseInBaseFrame(kinematics_solver_->getPose(r_gripper_name_));
            const Affine3d l_pose =
                computePoseInBaseFrame(kinematics_solver_->getPose(l_gripper_name_));
            const Affine3d r_elbow_pose =
                computePoseInBaseFrame(kinematics_solver_->getPose(r_elbow_name_));
            const Affine3d l_elbow_pose =
                computePoseInBaseFrame(kinematics_solver_->getPose(l_elbow_name_));
            r_goal_pose_pub_->publish(makePoseStamped(r_pose));
            l_goal_pose_pub_->publish(makePoseStamped(l_pose));
            r_elbow_pose_pub_->publish(makePoseStamped(r_elbow_pose));
            l_elbow_pose_pub_->publish(makePoseStamped(l_elbow_pose));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "FK computation failed: %s", e.what());
        }
    }

    geometry_msgs::msg::PoseStamped MiniLeaderController::makePoseStamped(const Affine3d& pose) const
    {
        geometry_msgs::msg::PoseStamped msg;
        // msg.header.stamp = this->now();
        msg.header.frame_id = base_frame_id_;
        msg.pose.position.x = pose.translation().x();
        msg.pose.position.y = pose.translation().y();
        msg.pose.position.z = pose.translation().z();

        const Eigen::Quaterniond quat(pose.linear());
        msg.pose.orientation.w = quat.w();
        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();
        return msg;
    }

    Affine3d MiniLeaderController::computePoseInBaseFrame(const Affine3d& link_pose) const
    {
        if (!base_frame_link_name_.empty() &&
            kinematics_solver_ &&
            kinematics_solver_->hasLinkFrame(base_frame_link_name_)) {
            const Affine3d base_pose = kinematics_solver_->getPose(base_frame_link_name_);
            return base_pose.inverse() * link_pose;
        }
        return link_pose;
    }
}  // namespace motion_controller_ros

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motion_controller_ros::MiniLeaderController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
