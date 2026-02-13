#pragma once

#include <Eigen/Dense>

namespace motion_controller
{
namespace common
{
// 6D twist / spatial vector
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace collision_checker
{
/**
 * @brief Result of minimum distance computation between links.
 *
 * Used for self-collision avoidance or proximity monitoring.
 * Contains the minimum distance and its sensitivity w.r.t. joint configuration.
 */
struct MinDistResult
{
    double distance{0.0};          // Minimum distance [m]
    Eigen::VectorXd grad;          // Gradient of distance w.r.t joint positions
    Eigen::VectorXd grad_dot;      // Time derivative of gradient

    void setZero(const int size)
    {
        distance = 0.0;
        grad.setZero(size);
        grad_dot.setZero(size);
    }
};
}  // namespace collision_checker

}  // namespace common
}  // namespace motion_controller

