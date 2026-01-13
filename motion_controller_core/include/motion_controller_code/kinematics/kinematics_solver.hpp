#ifndef MOTION_CONTROLLER_CORE_KINEMATICS_SOLVER_HPP_
#define MOTION_CONTROLLER_CORE_KINEMATICS_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace motion_controller_core
{

/**
 * @brief Enum to select the kinematics solver backend.
 */
enum class SolverType
{
  KDL,
  PINOCCHIO
};

// Forward declaration of the abstract base solver to hide implementation details.
class BaseSolver;

/**
 * @brief Generic Kinematics Solver class that provides FK and IK using a selectable backend.
 */
class KinematicsSolver
{
public:
  KinematicsSolver();
  ~KinematicsSolver();

  /**
   * @brief Initialize the kinematics solver.
   * 
   * @param urdf_content String containing the URDF model XML.
   * @param base_link Name of the base link of the chain.
   * @param tip_link Name of the tip link (end-effector) of the chain.
   * @param type The solver backend to use (KDL or PINOCCHIO).
   * @return true if initialization was successful, false otherwise.
   */
  bool init(const std::string& urdf_content, 
            const std::string& base_link, 
            const std::string& tip_link, 
            SolverType type);

  /**
   * @brief Solve Forward Kinematics.
   * 
   * @param q Joint positions (vector of size 'dof').
   * @param pose Output parameter for the computed end-effector pose.
   * @return true if solution found, false otherwise.
   */
  bool solveFK(const Eigen::VectorXd& q, Eigen::Isometry3d& pose);

  /**
   * @brief Solve Inverse Kinematics.
   * 
   * @param target_pose Desired end-effector pose.
   * @param q_init Initial guess for joint positions.
   * @param q_out Output parameter for the computed joint positions.
   * @return true if solution found/converged, false otherwise.
   */
  bool solveIK(const Eigen::Isometry3d& target_pose, const Eigen::VectorXd& q_init, Eigen::VectorXd& q_out);

private:
  std::unique_ptr<BaseSolver> solver_;
};

} // namespace motion_controller_core

#endif // MOTION_CONTROLLER_CORE_KINEMATICS_SOLVER_HPP_
