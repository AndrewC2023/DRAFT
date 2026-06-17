/*
 * Author: Andrew Campbell
 * Date: 06-13-2026
 */

#ifndef LINEARDYNAMICSBASE_H
#define LINEARDYNAMICSBASE_H

#include "DynamicsBase.hpp"

// Eigen
#include <Eigen/Dense>

// STD
#include <vector>
#include <string>


namespace Draft::Dynamics
{
    class LinearDynamics 
    {
        public:
            virtual ~LinearDynamics() = default;

            void GetSystemMatricies(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D) { A = A_; B = B_; C = C_; D = D_; };

        protected:
            // using state space model x_dot = Ax + Bu
            //                         y     = Cx + Du
            // Generally x is y so C and D are just identity but sometimes its different 
            Eigen::MatrixXd A_; // System Matrix
            Eigen::MatrixXd B_; // Input Matrix
            Eigen::MatrixXd C_; // Passthrough Matrix
            Eigen::MatrixXd D_; // Feedthrough Matrix
    };
}

#endif // LINEARDYNAMICSBASE_H