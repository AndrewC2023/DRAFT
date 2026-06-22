/*
 * Author: Andrew Campbell
 * Date: 06-13-2026
 */

#ifndef LINEARDYNAMICSBASE_H
#define LINEARDYNAMICSBASE_H

#include "DynamicsBase.hpp"

// Eigen
#include <Eigen/Dense>

// Internal math
#include "Util/Math/Math.hpp"

// STD
#include <stdexcept>
#include <utility>

namespace Draft::Dynamics
{
    class LinearDynamicsBase : public DynamicsBase
    {
        public:
            explicit LinearDynamicsBase(Eigen::MatrixXd A, Eigen::MatrixXd B)
                : A_(std::move(A)),
                  B_(std::move(B)),
                  C_(Eigen::MatrixXd::Identity(A_.rows(), A_.cols())),
                  D_(Eigen::MatrixXd::Zero(A_.rows(), B_.cols()))
            {
                ValidateSystemMatrices();
            }
            LinearDynamicsBase(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D)
                : A_(std::move(A)),
                  B_(std::move(B)),
                  C_(std::move(C)),
                  D_(std::move(D))
            {
                ValidateSystemMatrices();
            }

            using DynamicsBase::Step;
            void Step(double time, double dt, Eigen::VectorXd& state, const Eigen::VectorXd& systemInput, [[maybe_unused]] const Eigen::VectorXd& additionalInputs) const override
            {
                if (state.size() != A_.cols()){
                    throw std::invalid_argument("State dimension does not match A.");
                }
                if (systemInput.size() != B_.cols()){
                    throw std::invalid_argument("Input dimension does not match B.");
                }

                state = Draft::Util::Math::RungeKutta4(time, state, dt,
                    [this, &systemInput](double, const Eigen::VectorXd& evaluationState){
                        return A_ * evaluationState + B_ * systemInput;});
                
                ApplyConstraints(state);
            };

            const Eigen::MatrixXd& GetA() const noexcept { return A_; }
            const Eigen::MatrixXd& GetB() const noexcept { return B_; }
            const Eigen::MatrixXd& GetC() const noexcept { return C_; }
            const Eigen::MatrixXd& GetD() const noexcept { return D_; }
            void SetSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D) {
                    A_ = A; B_ = B; C_ = C; D_ = D;
                    ValidateSystemMatrices();
                };

        protected:
            // using state space model x_dot = Ax + Bu
            //                         y     = Cx + Du
            // Generally x is y so C and D are just identity but sometimes its different 
            Eigen::MatrixXd A_; // System Matrix
            Eigen::MatrixXd B_; // Input Matrix
            Eigen::MatrixXd C_; // Passthrough Matrix
            Eigen::MatrixXd D_; // Feedthrough Matrix

            void ValidateSystemMatrices() const
            {
                if (A_.rows() != A_.cols()){
                    throw std::invalid_argument("A must be square.");
                }
                if (B_.rows() != A_.rows()){
                    throw std::invalid_argument("B must have the same number of rows as A.");
                }
                if (C_.cols() != A_.cols()){
                    throw std::invalid_argument("C must have the same number of columns as A.");
                }
                if (D_.rows() != C_.rows() || D_.cols() != B_.cols()){
                    throw std::invalid_argument("D dimensions must match the output and input dimensions.");
                }
            }
    };
}

#endif // LINEARDYNAMICSBASE_H