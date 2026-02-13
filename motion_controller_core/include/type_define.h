#ifndef MATH_TYPE_DEFINE_H
#define MATH_TYPE_DEFINE_H

#include <Eigen/Dense>

using namespace Eigen;

// Define Vector6d as a 6D column vector
using Vector6d = Vector<double, 6>;

namespace CollisionChecker
{
    using Eigen::VectorXd;

	/**
     * @brief Result of minimum distance computation between links.
     *
     * Used for self-collision avoidance or proximity monitoring.
     * This structure contains the minimum distance between robot links and its sensitivity
     * (gradient and time derivative) with respect to the joint configuration.
     *
     * @param distance   (double)   Minimum distance between any two links (in meters).
     * @param grad       (VectorXd) Gradient of the minimum distance with respect to joint positions q.
     * @param grad_dot   (VectorXd) Time derivative of grad.
    */
    struct MinDistResult
    {
        double   distance;  // Minimum distance [m]
        VectorXd grad;      // Gradient of distance w.r.t. joint positions
        VectorXd grad_dot;  // Time derivative of gradient

        /**
         * @brief Initializes all fields to zero with given joint size.
         * @param size Number of joints
         */
        void setZero(const int size)
        {
            distance = 0;
            grad.setZero(size);
            grad_dot.setZero(size);
        }
    };
}
#endif