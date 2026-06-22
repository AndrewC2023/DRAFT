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
    /**
     * @brief Continuous-time linear state-space dynamics model.
     *
     * Implements x_dot = A x + B u and advances the caller-owned state with
     * fixed-step classical RK4 integration. C and D describe the optional
     * output equation y = C x + D u.
     */
    class LinearDynamicsBase : public DynamicsBase
    {
        public:
            /**
             * @brief Constructs a linear model with full-state output.
             *
             * C defaults to identity and D defaults to zero.
             *
             * @param A Continuous-time system matrix.
             * @param B Continuous-time input matrix.
             * @throws std::invalid_argument If the matrix dimensions are incompatible.
             */
            explicit LinearDynamicsBase(Eigen::MatrixXd A, Eigen::MatrixXd B)
                : A_(std::move(A)),
                  B_(std::move(B)),
                  C_(Eigen::MatrixXd::Identity(A_.rows(), A_.cols())),
                  D_(Eigen::MatrixXd::Zero(A_.rows(), B_.cols()))
            {
                ValidateSystemMatrices();
            }

            /**
             * @brief Constructs a complete continuous-time state-space model.
             *
             * @param A Continuous-time system matrix.
             * @param B Continuous-time input matrix.
             * @param C Output matrix.
             * @param D Feedthrough matrix.
             * @throws std::invalid_argument If the matrix dimensions are incompatible.
             */
            LinearDynamicsBase(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D)
                : A_(std::move(A)),
                  B_(std::move(B)),
                  C_(std::move(C)),
                  D_(std::move(D))
            {
                ValidateSystemMatrices();
            }

            using DynamicsBase::Step;

            /**
             * @brief Advances the linear state-space model by one timestep.
             *
             * Additional inputs are accepted for interface compatibility but
             * are not used by the generic linear model.
             *
             * @param time Time associated with the supplied state.
             * @param dt Positive integration timestep.
             * @param state State to advance in place.
             * @param systemInput Control input vector.
             * @param additionalInputs Unused additional-input vector.
             * @throws std::invalid_argument If state or input dimensions are invalid.
             */
            void Step(double time, double dt, Eigen::VectorXd& state, const Eigen::VectorXd& systemInput, [[maybe_unused]] const Eigen::VectorXd& additionalInputs) const override
            {
                state = Draft::Util::Math::RungeKutta4(time, state, dt,
                    [this, &systemInput](double, const Eigen::VectorXd& evaluationState){
                        return A_ * evaluationState + B_ * systemInput;});
                
                ApplyConstraints(state);
            };

            /** @brief Returns the system matrix A. */
            const Eigen::MatrixXd& GetA() const noexcept { return A_; }
            /** @brief Returns the input matrix B. */
            const Eigen::MatrixXd& GetB() const noexcept { return B_; }
            /** @brief Returns the output matrix C. */
            const Eigen::MatrixXd& GetC() const noexcept { return C_; }
            /** @brief Returns the feedthrough matrix D. */
            const Eigen::MatrixXd& GetD() const noexcept { return D_; }

            /**
             * @brief Replaces all state-space matrices.
             *
             * @param A Continuous-time system matrix.
             * @param B Continuous-time input matrix.
             * @param C Output matrix.
             * @param D Feedthrough matrix.
             * @throws std::invalid_argument If the matrix dimensions are incompatible.
             */
            void SetSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D) {
                    A_ = A; B_ = B; C_ = C; D_ = D;
                    ValidateSystemMatrices();
                };

        protected:
            Eigen::MatrixXd A_; ///< Continuous-time system matrix.
            Eigen::MatrixXd B_; ///< Continuous-time input matrix.
            Eigen::MatrixXd C_; ///< Output matrix.
            Eigen::MatrixXd D_; ///< Feedthrough matrix.

            /**
             * @brief Validates compatibility of the state-space matrices.
             *
             * @throws std::invalid_argument If any required dimensions differ.
             */
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
