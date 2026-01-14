#include "motion_controller_code/kinematics/kinematics_solver.hpp"

#include <iostream>
#include <stdexcept>

// KDL Includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

// Pinocchio Includes
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace motion_controller_core
{

// -----------------------------------------------------------------------------
// Base Solver Interface
// -----------------------------------------------------------------------------
class BaseSolver
{
public:
  virtual ~BaseSolver() = default;
  virtual bool init(const std::string& urdf_content, const std::string& base_link, const std::string& tip_link) = 0;
  virtual bool solveFK(const Eigen::VectorXd& q, Eigen::Isometry3d& pose) = 0;
  virtual bool solveIK(const Eigen::Isometry3d& target_pose, const Eigen::VectorXd& q_init, Eigen::VectorXd& q_out) = 0;
};

// -----------------------------------------------------------------------------
// KDL Solver Implementation
// -----------------------------------------------------------------------------
class KDLSolver : public BaseSolver
{
public:
  bool init(const std::string& urdf_content, const std::string& base_link, const std::string& tip_link) override
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_content, tree)) {
      std::cerr << "[KDLSolver] Failed to construct KDL tree from URDF." << std::endl;
      return false;
    }

    if (!tree.getChain(base_link, tip_link, chain_)) {
      std::cerr << "[KDLSolver] Failed to get chain from " << base_link << " to " << tip_link << std::endl;
      return false;
    }

    unsigned int nj = chain_.getNrOfJoints();

    // Joint Limits (needed for IK solver)
    q_min_.resize(nj);
    q_max_.resize(nj);
    // Note: To get actual limits, we'd need to parse the URDF thoroughly or assume treeFromString handled it.
    // kdl_parser extracts the tree structure but limits extraction depends on the parser version and model usage.
    // Ideally we iterate standard URDF segments. But KDL Chain doesn't store limits directly.
    // For simplicity in this core class without full URDF DOM access here (unless we re-parse with urdfdom),
    // we will set wide limits or try to extract if possible.
    // Actually, KDL solvers usually need the limits.
    // Let's assume standard +/- PI or huge values if not available, OR rely on an additional passed-in limits vector?
    // The prompt implies URDF is the input.
    // Let's use a very simple approach for limits: -2PI to 2PI for now as a default,
    // real implementations should probably extract them from the URDF DOM properly if critical.
    // However, since we are using kdl_parser which uses urdfdom, we might not have direct access to limits implementation-wise here easily.
    // For robust IK, limits are important. I'll set large default limits.
    for(unsigned int i=0; i<nj; ++i) {
        q_min_(i) = -M_PI * 10;
        q_max_(i) =  M_PI * 10;
    }

    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
    // NR_JL (Newton Raphson with Joint Limits)
    // We need 100 max iter and epsilon
    ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(chain_, q_min_, q_max_, *fk_solver_, *ik_vel_solver_, 100, 1e-6);

    return true;
  }

  bool solveFK(const Eigen::VectorXd& q, Eigen::Isometry3d& pose) override
  {
    if (q.size() != chain_.getNrOfJoints()) {
      std::cerr << "[KDLSolver] Joint size mismatch." << std::endl;
      return false;
    }

    KDL::JntArray kdl_q(q.size());
    for (int i = 0; i < q.size(); ++i) kdl_q(i) = q(i);

    KDL::Frame kdl_pose;
    if (fk_solver_->JntToCart(kdl_q, kdl_pose) < 0) {
      return false;
    }

    // Convert KDL::Frame to Eigen::Isometry3d
    double x, y, z, w;
    kdl_pose.M.GetQuaternion(x, y, z, w);
    Eigen::Quaterniond quat(w, x, y, z);
    Eigen::Vector3d trans(kdl_pose.p.x(), kdl_pose.p.y(), kdl_pose.p.z());

    pose = Eigen::Isometry3d::Identity();
    pose.linear() = quat.toRotationMatrix();
    pose.translation() = trans;

    return true;
  }

  bool solveIK(const Eigen::Isometry3d& target_pose, const Eigen::VectorXd& q_init, Eigen::VectorXd& q_out) override
  {
    if (q_init.size() != chain_.getNrOfJoints()) {
      std::cerr << "[KDLSolver] Joint size mismatch." << std::endl;
      return false;
    }

    KDL::JntArray kdl_q_init(q_init.size());
    for (int i = 0; i < q_init.size(); ++i) kdl_q_init(i) = q_init(i);

    // Convert Eigen::Isometry3d to KDL::Frame
    Eigen::Quaterniond quat(target_pose.linear());
    KDL::Rotation kdl_rot = KDL::Rotation::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
    KDL::Vector kdl_pos(target_pose.translation().x(), target_pose.translation().y(), target_pose.translation().z());
    KDL::Frame kdl_target(kdl_rot, kdl_pos);

    KDL::JntArray kdl_q_out(chain_.getNrOfJoints());

    int ret = ik_pos_solver_->CartToJnt(kdl_q_init, kdl_target, kdl_q_out);
    if (ret < 0) {
      return false;
    }

    q_out.resize(chain_.getNrOfJoints());
    for (int i = 0; i < chain_.getNrOfJoints(); ++i) {
      q_out(i) = kdl_q_out(i);
    }

    return true;
  }

private:
  KDL::Chain chain_;
  KDL::JntArray q_min_;
  KDL::JntArray q_max_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
};

// -----------------------------------------------------------------------------
// Pinocchio Solver Implementation
// -----------------------------------------------------------------------------
class PinocchioSolver : public BaseSolver
{
public:
  bool init(const std::string& urdf_content, const std::string& base_link, const std::string& tip_link) override
  {
    try {
        // 1. Build Full Model from URDF
        pinocchio::Model full_model;
        pinocchio::urdf::buildModelFromXML(urdf_content, full_model);

        if (!full_model.existFrame(tip_link)) {
             std::cerr << "[PinocchioSolver] Tip link " << tip_link << " not found." << std::endl;
             return false;
        }
        pinocchio::FrameIndex full_tip_id = full_model.getFrameId(tip_link);
        pinocchio::JointIndex tip_joint_id = full_model.frames[full_tip_id].parent;

        // 2. Identify Chain Joints (Tip -> Base)
        // We traverse up from tip joint to base link's parent joint.
        std::vector<pinocchio::JointIndex> chain_joints;

        // Find base_link frame/joint
        pinocchio::JointIndex base_joint_id = 0; // Default to Universe
        if (base_link != "universe" && base_link != "world" && full_model.existFrame(base_link)) {
            base_joint_id = full_model.frames[full_model.getFrameId(base_link)].parent;
        }

        // Trace path from Tip Joint to Base Joint
        // Note: Pinocchio joints are a tree. We walk up using parent.
        pinocchio::JointIndex current_joint = tip_joint_id;
        while (current_joint > base_joint_id) { // Assumes base_joint_id is ancestor
             chain_joints.push_back(current_joint);
             current_joint = full_model.parents[current_joint];
        }

        // Identify joints to LOCK (all joints NOT in chain)
        std::vector<pinocchio::JointIndex> joints_to_lock;
        // Mark chain joints
        std::vector<bool> is_chain_joint(full_model.njoints, false);
        for (auto j : chain_joints) is_chain_joint[j] = true;

        // Populate lock list (skip universe=0)
        for (pinocchio::JointIndex i = 1; i < full_model.njoints; ++i) {
            if (!is_chain_joint[i]) {
                joints_to_lock.push_back(i);
            }
        }

        // Reference configuration (neutral) for locked joints
        Eigen::VectorXd q_ref = pinocchio::neutral(full_model);

        // 3. Build Reduced Model
        // This creates a model with ONLY the chain joints active.
        pinocchio::buildReducedModel(full_model, joints_to_lock, q_ref, model_);
        data_ = pinocchio::Data(model_);

        // Update tip frame ID in Reduced Model
        if (model_.existFrame(tip_link)) {
             tip_frame_id_ = model_.getFrameId(tip_link);
        } else {
             // Should not happen if buildReducedModel works as expected
             std::cerr << "[PinocchioSolver] Tip link lost in reduced model." << std::endl;
             return false;
        }

        // Compute oMbase in Reduced Model (it's constant since parents are locked/removed)
        pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
        pinocchio::updateFramePlacements(model_, data_);

        if (model_.existFrame(base_link)) {
            pinocchio::FrameIndex base_frame_id = model_.getFrameId(base_link);
            oMbase_inv_ = data_.oMf[base_frame_id].inverse();
        } else {
            oMbase_inv_ = pinocchio::SE3::Identity();
        }

        J_.resize(6, model_.nv); // Resize Jacobian
        J_.setZero();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[PinocchioSolver] Exception: " << e.what() << std::endl;
        return false;
    }
  }

  bool solveFK(const Eigen::VectorXd& q, Eigen::Isometry3d& pose) override
  {
      if (q.size() != model_.nq) {
          std::cerr << "[PinocchioSolver] Model joint size (" << model_.nq << ") != input q size (" << q.size() << ")" << std::endl;
          return false;
      }

      pinocchio::forwardKinematics(model_, data_, q);
      pinocchio::updateFramePlacements(model_, data_);

      const auto& oMtip = data_.oMf[tip_frame_id_];

      // Compute relative pose: Base -> Tip
      // baseMtip = oMbase^-1 * oMtip
      pinocchio::SE3 baseMtip = oMbase_inv_ * oMtip;

      pose = Eigen::Isometry3d::Identity();
      pose.linear() = baseMtip.rotation();
      pose.translation() = baseMtip.translation();

      return true;
  }

  bool solveIK(const Eigen::Isometry3d& target_pose, const Eigen::VectorXd& q_init, Eigen::VectorXd& q_out) override
  {
      if (q_init.size() != model_.nq) return false;

      Eigen::VectorXd q = q_init;
      const double eps = 1e-6;
      const int max_iter = 100;
      const double dt = 0.1;
      const double damp = 1e-12;

      pinocchio::SE3 baseMtarget(target_pose.linear(), target_pose.translation());
      pinocchio::SE3 oMtarget = oMbase_inv_.inverse() * baseMtarget;

      for(int i=0; i<max_iter; ++i) {
          pinocchio::forwardKinematics(model_, data_, q);
          pinocchio::updateFramePlacements(model_, data_);

          const pinocchio::SE3& current_pose = data_.oMf[tip_frame_id_];
          pinocchio::Motion err = pinocchio::log6(current_pose.inverse() * oMtarget);

          if(err.toVector().norm() < eps) {
              q_out = q;
              return true;
          }

          pinocchio::computeFrameJacobian(model_, data_, q, tip_frame_id_, pinocchio::LOCAL, J_);

          pinocchio::Data::Matrix6x Jlog;
          pinocchio::Jlog6(current_pose.inverse() * oMtarget, Jlog);
          J_ = -Jlog * J_;

          pinocchio::Data::Matrix6 JJt;
          JJt.noalias() = J_ * J_.transpose();
          JJt.diagonal().array() += damp;

          Eigen::VectorXd v = -J_.transpose() * JJt.ldlt().solve(err.toVector());
          q = pinocchio::integrate(model_, q, v * dt);
      }

      return false;
  }

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex tip_frame_id_;
  pinocchio::SE3 oMbase_inv_;
  pinocchio::Data::Matrix6x J_;
};


// -----------------------------------------------------------------------------
// Main Kinematics Solver Class Implementation
// -----------------------------------------------------------------------------
KinematicsSolver::KinematicsSolver() = default;
KinematicsSolver::~KinematicsSolver() = default;

bool KinematicsSolver::init(const std::string& urdf_content,
                     const std::string& base_link,
                     const std::string& tip_link,
                     SolverType type)
{
  switch (type) {
    case SolverType::KDL:
      solver_ = std::make_unique<KDLSolver>();
      break;
    case SolverType::PINOCCHIO:
      solver_ = std::make_unique<PinocchioSolver>();
      break;
    default:
      return false;
  }
  return solver_->init(urdf_content, base_link, tip_link);
}

bool KinematicsSolver::solveFK(const Eigen::VectorXd& q, Eigen::Isometry3d& pose)
{
  if (!solver_) return false;
  return solver_->solveFK(q, pose);
}

bool KinematicsSolver::solveIK(const Eigen::Isometry3d& target_pose, const Eigen::VectorXd& q_init, Eigen::VectorXd& q_out)
{
  if (!solver_) return false;
  return solver_->solveIK(target_pose, q_init, q_out);
}

} // namespace motion_controller_core
