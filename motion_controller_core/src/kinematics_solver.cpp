#include "motion_controller_core/kinematics_solver.hpp"

#include <iostream>
#include <stdexcept>

namespace motion_controller_core
{
    KinematicsSolver::KinematicsSolver(const std::string& urdf_path)
    : urdf_path_(urdf_path)
    {
        if (!std::filesystem::exists(urdf_path))
        {
            throw std::runtime_error("URDF file does not exist: " + urdf_path);
        }

        pinocchio::urdf::buildModel(urdf_path, model_, /*verbose=*/false);
        data_ = pinocchio::Data(model_);

        dof_ = model_.joints.size() - 1; // except world joint

        // Initialize joint space state
        q_.setZero(dof_);
        qdot_.setZero(dof_);

        // Set joint state limits
        q_lb_ = model_.lowerPositionLimit;
        q_ub_ = model_.upperPositionLimit;
        qdot_lb_ = -model_.velocityLimit;
        qdot_ub_ = model_.velocityLimit;

        // Clear frame collections
        link_frame_names_.clear();
        joint_frame_names_.clear();
        link_frame_set_.clear();
        joint_frame_set_.clear();
        root_link_name_.clear();

        // Collect frames
        for (pinocchio::FrameIndex i = 0; i < model_.frames.size(); ++i)
        {
            const auto& f = model_.frames[i];

            // LINK frames (URDF <link>)
            if (f.type == pinocchio::FrameType::BODY && f.name != "universe")
            {
                link_frame_names_.push_back(f.name);
                link_frame_set_.insert(f.name);

                // Root link: BODY directly attached to universe joint
                if (f.parentJoint == 0 && root_link_name_.empty())
                {
                    root_link_name_ = f.name;
                }
            }
            // JOINT frames (URDF <joint>)
            else if (f.type == pinocchio::FrameType::JOINT)
            {
                joint_frame_names_.push_back(f.name);
                joint_frame_set_.insert(f.name);
            }
        }

        if (link_frame_names_.empty())
        {
            throw std::runtime_error("No BODY frames found in model");
        }

        // Fallback root link
        if (root_link_name_.empty())
        {
            root_link_name_ = link_frame_names_.front();
        }
    }

    KinematicsSolver::~KinematicsSolver()
    {
        // Cleanup if needed - pinocchio objects handle their own cleanup
    }

    bool KinematicsSolver::updateState(const VectorXd& q, const VectorXd& qdot)
    {
        q_ = q;
        qdot_ = qdot;
        
        if(!updateKinematics(q_, qdot_)) return false;
        return true;
    }

    bool KinematicsSolver::updateKinematics(const VectorXd& q, const VectorXd& qdot)
    {
        pinocchio::computeJointJacobians(model_, data_, q);
        
        return true;
    }

    // ================================ Compute Functions ================================

    Affine3d KinematicsSolver::computePose(const VectorXd& q, const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))
        {
            throw std::runtime_error("Link name not found in URDF: " + link_name);
        }
        
        pinocchio::Data data = pinocchio::Data(model_);
        pinocchio::framesForwardKinematics(model_, data, q);
        Affine3d link_pose;
        link_pose.matrix() = data.oMf[link_index].toHomogeneousMatrix();
        
        return link_pose;
    }

    MatrixXd KinematicsSolver::computeJacobian(const VectorXd& q, const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))
        {
            throw std::runtime_error("Link name not found in URDF: " + link_name);
        }
        
        MatrixXd J;
        J.setZero(6, dof_);
        pinocchio::Data data = pinocchio::Data(model_);
        pinocchio::computeJointJacobians(model_, data, q);
        pinocchio::computeFrameJacobian(model_, data, q, link_index, 
                                        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
    
        return J;
    }

    // bool solveIK(const Isometry3d& target_pose, const VectorXd& q_init, VectorXd& q_out) override
    // {
    //     if (q_init.size() != model_.nq) return false;
        
    //     VectorXd q = q_init;
    //     const double eps = 1e-6;
    //     const int max_iter = 100;
    //     const double dt = 0.1; 
    //     const double damp = 1e-12;
        
    //     pinocchio::SE3 baseMtarget(target_pose.linear(), target_pose.translation());
    //     pinocchio::SE3 oMtarget = oMbase_inv_.inverse() * baseMtarget;
        
    //     for(int i=0; i<max_iter; ++i) {
    //         pinocchio::forwardKinematics(model_, data_, q);
    //         pinocchio::updateFramePlacements(model_, data_);
            
    //         const pinocchio::SE3& current_pose = data_.oMf[tip_frame_id_];
    //         pinocchio::Motion err = pinocchio::log6(current_pose.inverse() * oMtarget);
            
    //         if(err.toVector().norm() < eps) {
    //             q_out = q;
    //             return true;
    //         }
            
    //         pinocchio::computeFrameJacobian(model_, data_, q, tip_frame_id_, pinocchio::LOCAL, J_);
            
    //         pinocchio::Data::Matrix6x Jlog;
    //         pinocchio::Jlog6(current_pose.inverse() * oMtarget, Jlog);
    //         J_ = -Jlog * J_;
            
    //         pinocchio::Data::Matrix6 JJt;
    //         JJt.noalias() = J_ * J_.transpose();
    //         JJt.diagonal().array() += damp;
            
    //         VectorXd v = -J_.transpose() * JJt.ldlt().solve(err.toVector());
    //         q = pinocchio::integrate(model_, q, v * dt);
    //     }
        
    //     return false;
    // }

    // ================================ Get Functions ================================
    bool KinematicsSolver::hasLinkFrame(const std::string& name) const
    {
        return link_frame_set_.count(name) > 0;
    }

    bool KinematicsSolver::hasJointFrame(const std::string& name) const
    {
        return joint_frame_set_.count(name) > 0;
    }

    std::vector<std::string> KinematicsSolver::getJointNames() const
    {
        // Return the joint frame names we collected from the URDF
        // These should match the joint names in the joint_states topic
        return joint_frame_names_;
    }

    Affine3d KinematicsSolver::getPose(const std::string& link_name) const
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))
        {
            throw std::runtime_error("Link name not found in URDF: " + link_name);
        }
        
        Affine3d link_pose;
        link_pose.matrix() = data_.oMf[link_index].toHomogeneousMatrix();
    
        return link_pose;
    }
        
    MatrixXd KinematicsSolver::getJacobian(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))
        {
            throw std::runtime_error("Link name not found in URDF: " + link_name);
        }
    
        return pinocchio::getFrameJacobian(model_, data_, link_index, 
                                           pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    }

} // namespace motion_controller_core
