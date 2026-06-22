#ifndef LQR_H
#define LQR_H

// Eigen
#include <Eigen/Dense>

namespace Draft::Autonomy::Control
{
    /**
     * @brief Continuous-time infinite-horizon linear-quadratic regulator.
     *
     * The controller solves the continuous-time algebraic Riccati equation
     *
     * @f[
     * A^T P + P A - P B R^{-1} B^T P + Q = 0
     * @f]
     *
     * and calculates the state-feedback gain
     *
     * @f[
     * K = R^{-1} B^T P.
     * @f]
     *
     * The stabilizing solution is recovered from the stable invariant
     * subspace of the associated Hamiltonian matrix. This implementation uses
     * a direct eigendecomposition of that subspace rather than an ordered Schur
     * decomposition.
     *
     * @par Reference
     * A. J. Laub, "A Schur Method for Solving Algebraic Riccati Equations,"
     * IEEE Transactions on Automatic Control, vol. 24, no. 6, pp. 913-921,
     * 1979. DOI: 10.1109/TAC.1979.1102178.
     *
     * @see https://doi.org/10.1109/TAC.1979.1102178
     */
    class LQR
    {
        public:
            /**
             * @brief Constructs an LQR controller and computes its gain.
             *
             * @param A Continuous-time system matrix.
             * @param B Continuous-time input matrix.
             * @param Q Positive-semidefinite state-cost matrix.
             * @param R Positive-definite input-cost matrix.
             * @throws std::invalid_argument If Q or R does not satisfy the
             * required symmetry or definiteness conditions.
             * @throws std::runtime_error If CARE cannot be computed or its
             * final residual is too large.
             */
            LQR(
                Eigen::MatrixXd A,
                Eigen::MatrixXd B,
                Eigen::MatrixXd Q,
                Eigen::MatrixXd R);

            /**
             * @brief Computes reference-tracking state feedback.
             *
             * Implements @f$u = u_0 - K(x-x_0)@f$.
             *
             * @param state Current state vector.
             * @param referenceState Reference or trim state vector.
             * @param referenceInput Reference or trim input vector.
             * @return Control input vector.
             */
            Eigen::VectorXd ComputeControl(const Eigen::VectorXd& state, const Eigen::VectorXd& referenceState, const Eigen::VectorXd& referenceInput) const;

            /** @brief Returns the state-feedback gain K. */
            const Eigen::MatrixXd& GetGain() const noexcept { return K_; }

            /** @brief Returns the stabilizing CARE solution P. */
            const Eigen::MatrixXd& GetRiccatiSolution() const noexcept { return P_; }

            /** @brief Returns the Frobenius norm of the final CARE residual. */
            double GetCareResidualNorm() const noexcept { return careResidualNorm_; }

        private:
            /**
             * @brief Validates symmetry and definiteness of Q and R.
             *
             * @throws std::invalid_argument If Q is not symmetric positive
             * semidefinite or R is not symmetric positive definite.
             */
            void ValidateCostMatrices() const;

            /**
             * @brief Solves CARE and computes the stabilizing feedback gain.
             *
             * Uses the stable invariant subspace of the CARE Hamiltonian
             * matrix following the construction described by Laub (1979).
             *
             * @see https://doi.org/10.1109/TAC.1979.1102178
             */
            void SolveCare();

            Eigen::MatrixXd A_; ///< Continuous-time system matrix.
            Eigen::MatrixXd B_; ///< Continuous-time input matrix.
            Eigen::MatrixXd Q_; ///< State-cost matrix.
            Eigen::MatrixXd R_; ///< Input-cost matrix.
            Eigen::MatrixXd P_; ///< Stabilizing CARE solution.
            Eigen::MatrixXd K_; ///< State-feedback gain.
            double careResidualNorm_ = 0.0; ///< Final CARE residual norm.
    };
}

#endif // LQR_H
