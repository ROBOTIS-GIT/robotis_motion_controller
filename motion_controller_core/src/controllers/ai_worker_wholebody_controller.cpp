#include "motion_controller_core/controllers/ai_worker_wholebody_controller.hpp"
#include <algorithm>
#include <cmath>

namespace motion_controller
{
namespace controllers
{

        WholebodyQPIK::WholebodyQPIK(std::shared_ptr<motion_controller::kinematics::KinematicsSolver> robot_data, const double dt)
        : motion_controller::optimization::QPBase(), robot_data_(robot_data), dt_(dt)
        {
            joint_dof_ = robot_data_->getDof();
    
            si_index_.qdot_size            = joint_dof_;
            si_index_.slack_q_min_size     = joint_dof_;
            si_index_.slack_q_max_size     = joint_dof_;
            si_index_.slack_sing_size      = 1;
            si_index_.slack_sel_col_size   = robot_data_->getCollisionPairCount();
            si_index_.con_q_min_size       = joint_dof_;
            si_index_.con_q_max_size       = joint_dof_;
            si_index_.con_sing_size        = 1;
            si_index_.con_sel_col_size     = robot_data_->getCollisionPairCount();
            si_index_.con_base_accel_size  = 3;  // [vx_body, vy_body, wz]
            
            const int nx = si_index_.qdot_size +
                           si_index_.slack_q_min_size +
                           si_index_.slack_q_max_size +
                           si_index_.slack_sing_size +
                           si_index_.slack_sel_col_size;
            const int nbound = nx;
            const int nineq = si_index_.con_q_min_size +
                              si_index_.con_q_max_size +
                              si_index_.con_sing_size +
                              si_index_.con_sel_col_size +
                              si_index_.con_base_accel_size;
            const int neq = 0;
            
            QPBase::setQPsize(nx, nbound, nineq, neq);
            
            si_index_.qdot_start          = 0;
            si_index_.slack_q_min_start   = si_index_.qdot_start + si_index_.qdot_size;
            si_index_.slack_q_max_start   = si_index_.slack_q_min_start + si_index_.slack_q_min_size;
            si_index_.slack_sing_start    = si_index_.slack_q_max_start + si_index_.slack_q_max_size;
            si_index_.slack_sel_col_start = si_index_.slack_sing_start + si_index_.slack_sing_size;
            si_index_.con_q_min_start     = 0;
            si_index_.con_q_max_start     = si_index_.con_q_min_start + si_index_.con_q_min_size;
            si_index_.con_sing_start      = si_index_.con_q_max_start + si_index_.con_q_max_size;
            si_index_.con_sel_col_start   = si_index_.con_sing_start + si_index_.con_sing_size;
            si_index_.con_base_accel_start = si_index_.con_sel_col_start + si_index_.con_sel_col_size;

            w_damping_.setOnes(joint_dof_);

            // Setup base joint indices for lag-comp (wholebody model)
            qdot_cmd_gain_.setOnes(joint_dof_);
            const auto joint_names = robot_data_->getJointNames();
            margin_exclude_mask_.assign(static_cast<size_t>(joint_dof_), 0);
            for (size_t i = 0; i < joint_names.size(); ++i) {
                const auto &name = joint_names[i];
                if (name == "x_joint") {
                    base_x_joint_index_ = static_cast<int>(i);
                } else if (name == "y_joint") {
                    base_y_joint_index_ = static_cast<int>(i);
                } else if (name == "pivot_joint") {
                    base_pivot_joint_index_ = static_cast<int>(i);
                }

                // Exclude non-arm joints from joint-limit margin signal (best-effort by name).
                if (name == "x_joint" || name == "y_joint" || name == "pivot_joint" ||
                    name.find("lift_joint") != std::string::npos ||
                    name.find("gripper") != std::string::npos)
                {
                    if (i < margin_exclude_mask_.size()) {
                        margin_exclude_mask_[i] = 1;
                    }
                }
            }
            updateActuationGains_();
        }

        void WholebodyQPIK::enableBaseActuationLagComp(const bool enabled)
        {
            base_actuation_lag_comp_enabled_ = enabled;
            updateActuationGains_();
        }

        void WholebodyQPIK::setBaseActuationTimeConstant(const double tau_seconds)
        {
            base_actuation_tau_seconds_ = tau_seconds;
            updateActuationGains_();
        }

        void WholebodyQPIK::updateActuationGains_()
        {
            qdot_cmd_gain_.setOnes(joint_dof_);
            if (!base_actuation_lag_comp_enabled_) {
                return;
            }

            // Discrete gain for 1st-order velocity tracking: gain = 1 - exp(-dt/tau)
            double gain = 1.0;
            if (base_actuation_tau_seconds_ > 0.0) {
                gain = 1.0 - std::exp(-dt_ / base_actuation_tau_seconds_);
            }
            gain = std::clamp(gain, 0.0, 1.0);

            auto set_gain = [this, gain](int idx) {
                if (idx >= 0 && idx < joint_dof_) {
                    qdot_cmd_gain_[idx] = gain;
                }
            };
            set_gain(base_x_joint_index_);
            set_gain(base_y_joint_index_);
            set_gain(base_pivot_joint_index_);
        }
    
        void WholebodyQPIK::setDesiredTaskVel(const std::map<std::string, motion_controller::common::Vector6d> &link_xdot_desired)
        {
            link_xdot_desired_ = link_xdot_desired;
        }
    
        bool WholebodyQPIK::getOptJointVel(Eigen::VectorXd &opt_qdot)
        {
            Eigen::MatrixXd sol;
            if(!solveQP(sol))
            {
                opt_qdot.setZero();
                return false;
            }
            else
            {
                opt_qdot = sol.block(si_index_.qdot_start,0,si_index_.qdot_size,1);

                // Update previous base command in body frame for acceleration constraints
                if (base_accel_cfg_.enabled &&
                    base_x_joint_index_ >= 0 && base_y_joint_index_ >= 0 && base_pivot_joint_index_ >= 0 &&
                    base_x_joint_index_ < opt_qdot.size() &&
                    base_y_joint_index_ < opt_qdot.size() &&
                    base_pivot_joint_index_ < opt_qdot.size())
                {
                    const Eigen::VectorXd q = robot_data_->getJointPosition();
                    const double yaw = (base_pivot_joint_index_ < q.size()) ? q[base_pivot_joint_index_] : 0.0;
                    const double c = std::cos(yaw);
                    const double s = std::sin(yaw);
                    const double xdot_world = opt_qdot[base_x_joint_index_];
                    const double ydot_world = opt_qdot[base_y_joint_index_];
                    prev_base_cmd_body_.x() = c * xdot_world + s * ydot_world;
                    prev_base_cmd_body_.y() = -s * xdot_world + c * ydot_world;
                    prev_base_cmd_body_.z() = opt_qdot[base_pivot_joint_index_];
                    prev_base_cmd_initialized_ = true;
                }

                return true;
            }
        }

        void WholebodyQPIK::setWeight(
            const std::map<std::string, motion_controller::common::Vector6d> link_w_tracking,
            const Eigen::VectorXd w_damping)
        {
            link_w_tracking_ = link_w_tracking;
            w_damping_ = w_damping;
        }

        void WholebodyQPIK::setControllerParams(const double slack_penalty, const double cbf_alpha, const double buffer_distance, const double safe_distance)
        {
            slack_penalty_ = slack_penalty;
            cbf_alpha_ = cbf_alpha;
            collision_buffer_ = buffer_distance;
            collision_safe_distance_ = safe_distance;
        }
    
        void WholebodyQPIK::setCost()
        {
            P_ds_.setZero(nx_, nx_);
            q_ds_.setZero(nx_);

            // Effective joint velocity model for task tracking:
            // qdot_eff = gain .* qdot_cmd + (1-gain) .* qdot_meas
            const Eigen::VectorXd qdot_meas = robot_data_->getJointVelocity();
            const Eigen::VectorXd one_minus_gain = (Eigen::VectorXd::Ones(joint_dof_) - qdot_cmd_gain_);
            const Eigen::VectorXd qdot_offset = one_minus_gain.cwiseProduct(qdot_meas);  // constant term

            // for task space velocity tracking
            for(const auto& [link_name, xdot_desired] : link_xdot_desired_)
            {
                Eigen::MatrixXd J_i = robot_data_->getJacobian(link_name);
                motion_controller::common::Vector6d w_tracking = motion_controller::common::Vector6d::Ones();

                auto iter = link_w_tracking_.find(link_name);
                if(iter != link_w_tracking_.end()) {
                    w_tracking = iter->second;
                }

                // Apply actuation gain on decision variables (column-wise scaling)
                const Eigen::MatrixXd J_eff = J_i * qdot_cmd_gain_.asDiagonal();
                const motion_controller::common::Vector6d xdot_offset = J_i * qdot_offset;

                P_ds_.block(si_index_.qdot_start, si_index_.qdot_start, si_index_.qdot_size, si_index_.qdot_size) +=
                    2.0 * J_eff.transpose() * w_tracking.asDiagonal() * J_eff;
                q_ds_.segment(si_index_.qdot_start, si_index_.qdot_size) +=
                    -2.0 * J_eff.transpose() * w_tracking.asDiagonal() * (xdot_desired - xdot_offset);
            }
            
            // for joint velocity damping
            P_ds_.block(si_index_.qdot_start, si_index_.qdot_start, si_index_.qdot_size, si_index_.qdot_size) +=
                2.0 * w_damping_.asDiagonal();
            
            // for slack variables
            q_ds_.segment(si_index_.slack_q_min_start, si_index_.slack_q_min_size) =
                Eigen::VectorXd::Constant(si_index_.slack_q_min_size, slack_penalty_);
            q_ds_.segment(si_index_.slack_q_max_start, si_index_.slack_q_max_size) =
                Eigen::VectorXd::Constant(si_index_.slack_q_max_size, slack_penalty_);
            q_ds_(si_index_.slack_sing_start) = slack_penalty_;
            if (si_index_.slack_sel_col_size > 0) {
                q_ds_.segment(si_index_.slack_sel_col_start, si_index_.slack_sel_col_size) =
                    Eigen::VectorXd::Constant(si_index_.slack_sel_col_size, slack_penalty_);
            }
        }
    
        void WholebodyQPIK::setBoundConstraint()
        {
            l_bound_ds_.setConstant(nbc_, -OSQP_INFTY);
            u_bound_ds_.setConstant(nbc_, OSQP_INFTY);

            // Manipulator Joint Velocity Limit
            l_bound_ds_.segment(si_index_.qdot_start, si_index_.qdot_size) = 
                robot_data_->getJointVelocityLimit().first;
            u_bound_ds_.segment(si_index_.qdot_start, si_index_.qdot_size) = 
                robot_data_->getJointVelocityLimit().second;

            // Slack variables must be non-negative
            l_bound_ds_.segment(si_index_.slack_q_min_start, si_index_.slack_q_min_size).setZero();
            l_bound_ds_.segment(si_index_.slack_q_max_start, si_index_.slack_q_max_size).setZero();
            l_bound_ds_(si_index_.slack_sing_start) = 0.0;
            if (si_index_.slack_sel_col_size > 0) {
                l_bound_ds_.segment(si_index_.slack_sel_col_start, si_index_.slack_sel_col_size).setZero();
            }
        }
    
        void WholebodyQPIK::setIneqConstraint()
        {
            A_ineq_ds_.setZero(nineqc_, nx_);
            l_ineq_ds_.setConstant(nineqc_, -OSQP_INFTY);
            u_ineq_ds_.setConstant(nineqc_, OSQP_INFTY);

            // Manipulator Joint Angle Limit (CBF)
            const Eigen::VectorXd q_min = robot_data_->getJointPositionLimit().first;
            const Eigen::VectorXd q_max = robot_data_->getJointPositionLimit().second;
            const Eigen::VectorXd q = robot_data_->getJointPosition();
                
            // Lower bound constraint: qdot + slack >= -alpha*(q - q_min)
            A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.qdot_start, 
                            si_index_.con_q_min_size, si_index_.qdot_size) = 
                Eigen::MatrixXd::Identity(si_index_.con_q_min_size, si_index_.qdot_size);
            A_ineq_ds_.block(si_index_.con_q_min_start, si_index_.slack_q_min_start, 
                            si_index_.con_q_min_size, si_index_.slack_q_min_size) = 
                Eigen::MatrixXd::Identity(si_index_.con_q_min_size, si_index_.slack_q_min_size);
            l_ineq_ds_.segment(si_index_.con_q_min_start, si_index_.con_q_min_size) = 
                -cbf_alpha_ * (q - q_min);
            
            // Upper bound constraint: -qdot + slack >= -alpha*(q_max - q)
            A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.qdot_start, 
                            si_index_.con_q_max_size, si_index_.qdot_size) = 
                -Eigen::MatrixXd::Identity(si_index_.con_q_max_size, si_index_.qdot_size);
            A_ineq_ds_.block(si_index_.con_q_max_start, si_index_.slack_q_max_start, 
                            si_index_.con_q_max_size, si_index_.slack_q_max_size) = 
                Eigen::MatrixXd::Identity(si_index_.con_q_max_size, si_index_.slack_q_max_size);
            l_ineq_ds_.segment(si_index_.con_q_max_start, si_index_.con_q_max_size) = 
                -cbf_alpha_ * (q_max - q);

            // self collision avoidance (CBF)
            if (si_index_.con_sel_col_size > 0)
            {
                const auto pair_results = robot_data_->getCollisionPairDistances(true, false, false);
                const int pair_count = std::min<int>(si_index_.con_sel_col_size, pair_results.size());
                for (int i = 0; i < pair_count; ++i)
                {
                    const auto &res = pair_results[i];
                    A_ineq_ds_.block(si_index_.con_sel_col_start + i, si_index_.qdot_start, 1, si_index_.qdot_size) =
                        res.grad.transpose();
                    if (si_index_.slack_sel_col_size > 0 && i < si_index_.slack_sel_col_size) {
                        A_ineq_ds_(si_index_.con_sel_col_start + i, si_index_.slack_sel_col_start + i) = 1.0;
                    }
                    if (res.distance <= collision_buffer_)
                    {
                        l_ineq_ds_(si_index_.con_sel_col_start + i) =
                            -cbf_alpha_ * (res.distance - collision_safe_distance_);
                    }
                }
            }

            // Base acceleration constraint (body-frame): limit delta-v each cycle
            // Constraints are of form: l <= A*qdot <= u
            // where A maps world (x_joint,y_joint) velocities to body vx/vy using current yaw.
            if (si_index_.con_base_accel_size > 0 &&
                base_accel_cfg_.enabled &&
                base_x_joint_index_ >= 0 && base_y_joint_index_ >= 0 && base_pivot_joint_index_ >= 0 &&
                base_x_joint_index_ < joint_dof_ && base_y_joint_index_ < joint_dof_ && base_pivot_joint_index_ < joint_dof_)
            {
                const Eigen::VectorXd q = robot_data_->getJointPosition();
                const double yaw = (base_pivot_joint_index_ < q.size()) ? q[base_pivot_joint_index_] : 0.0;
                const double c = std::cos(yaw);
                const double s = std::sin(yaw);

                Eigen::Vector3d prev_body = prev_base_cmd_body_;
                if (!prev_base_cmd_initialized_) {
                    const Eigen::VectorXd qdot_meas = robot_data_->getJointVelocity();
                    const double xdot_world_meas = (base_x_joint_index_ < qdot_meas.size()) ? qdot_meas[base_x_joint_index_] : 0.0;
                    const double ydot_world_meas = (base_y_joint_index_ < qdot_meas.size()) ? qdot_meas[base_y_joint_index_] : 0.0;
                    prev_body.x() = c * xdot_world_meas + s * ydot_world_meas;
                    prev_body.y() = -s * xdot_world_meas + c * ydot_world_meas;
                    prev_body.z() = (base_pivot_joint_index_ < qdot_meas.size()) ? qdot_meas[base_pivot_joint_index_] : 0.0;
                }

                const int row_vx = si_index_.con_base_accel_start + 0;
                const int row_vy = si_index_.con_base_accel_start + 1;
                const int row_wz = si_index_.con_base_accel_start + 2;

                // vx_body = c*xdot_world + s*ydot_world
                A_ineq_ds_(row_vx, si_index_.qdot_start + base_x_joint_index_) = c;
                A_ineq_ds_(row_vx, si_index_.qdot_start + base_y_joint_index_) = s;

                // vy_body = -s*xdot_world + c*ydot_world
                A_ineq_ds_(row_vy, si_index_.qdot_start + base_x_joint_index_) = -s;
                A_ineq_ds_(row_vy, si_index_.qdot_start + base_y_joint_index_) = c;

                // wz = pivot joint velocity
                A_ineq_ds_(row_wz, si_index_.qdot_start + base_pivot_joint_index_) = 1.0;

                auto set_bounds = [this](int row, double prev, double accel_max) {
                    if (accel_max <= 0.0) {
                        l_ineq_ds_(row) = -OSQP_INFTY;
                        u_ineq_ds_(row) = OSQP_INFTY;
                        return;
                    }
                    const double dv = accel_max * dt_;
                    l_ineq_ds_(row) = prev - dv;
                    u_ineq_ds_(row) = prev + dv;
                };

                set_bounds(row_vx, prev_body.x(), base_accel_cfg_.accel_vx_max);
                set_bounds(row_vy, prev_body.y(), base_accel_cfg_.accel_vy_max);
                set_bounds(row_wz, prev_body.z(), base_accel_cfg_.accel_wz_max);
            }
        }
    
        void WholebodyQPIK::setEqConstraint()
        {
            A_eq_ds_.setZero(neqc_, nx_);
            b_eq_ds_.setZero(neqc_);
        }

} // namespace controllers
} // namespace motion_controller
