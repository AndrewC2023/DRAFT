#include "LQR.hpp"

// Eigen
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

// STL
#include <stdexcept>
#include <utility>

namespace Draft::Autonomy::Control
{
    LQR::LQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R)
        : A_(std::move(A)),
          B_(std::move(B)),
          Q_(std::move(Q)),
          R_(std::move(R))
    {
        ValidateCostMatrices();
        SolveCare();
    }

    Eigen::VectorXd LQR::ComputeControl(const Eigen::VectorXd& state, const Eigen::VectorXd& referenceState, const Eigen::VectorXd& referenceInput) const
    {
        return referenceInput - K_ * (state - referenceState);
    }

    void LQR::ValidateCostMatrices() const
    {
        constexpr double SymmetryTolerance = 1e-10;
        if ((Q_ - Q_.transpose()).norm()
            > SymmetryTolerance * (1.0 + Q_.norm())){
            throw std::invalid_argument("Q must be symmetric.");
        }
        if ((R_ - R_.transpose()).norm()
            > SymmetryTolerance * (1.0 + R_.norm())){
            throw std::invalid_argument("R must be symmetric.");
        }

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> qSolver(Q_);
        if (qSolver.eigenvalues().minCoeff() < -1e-10){
            throw std::invalid_argument("Q must be positive semidefinite.");
        }

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> rSolver(R_);
        if (rSolver.eigenvalues().minCoeff() <= 0.0){
            throw std::invalid_argument("R must be positive definite.");
        }
    }

    void LQR::SolveCare()
    {
        const Eigen::Index stateDimension = A_.rows();

        // Solve with R instead of explicitly constructing R inverse.
        Eigen::LDLT<Eigen::MatrixXd> rSolver(R_);
        const Eigen::MatrixXd rInverseBT = rSolver.solve(B_.transpose());

        // CARE's stabilizing solution is encoded in the stable invariant
        // subspace of this Hamiltonian matrix.
        Eigen::MatrixXd hamiltonian = Eigen::MatrixXd::Zero(2 * stateDimension, 2 * stateDimension);
        hamiltonian.topLeftCorner(stateDimension, stateDimension) = A_;
        hamiltonian.topRightCorner(stateDimension, stateDimension) = -B_ * rInverseBT;
        hamiltonian.bottomLeftCorner(stateDimension, stateDimension) = -Q_;
        hamiltonian.bottomRightCorner(stateDimension, stateDimension) = -A_.transpose();

        Eigen::ComplexEigenSolver<Eigen::MatrixXd> eigenSolver(hamiltonian);

        Eigen::MatrixXcd stableEigenvectors(2 * stateDimension, stateDimension);
        Eigen::Index stableCount = 0;
        for (Eigen::Index ii = 0; ii < eigenSolver.eigenvalues().size(); ii++){
            if (eigenSolver.eigenvalues()(ii).real() < -1e-10){
                if (stableCount < stateDimension){
                    stableEigenvectors.col(stableCount) = eigenSolver.eigenvectors().col(ii);
                    stableCount++;
                }
            }
        }

        if (stableCount != stateDimension){
            throw std::runtime_error("CARE Hamiltonian does not have the required stable subspace.");
        }

        const Eigen::MatrixXcd upperEigenvectors = stableEigenvectors.topRows(stateDimension);
        const Eigen::MatrixXcd lowerEigenvectors = stableEigenvectors.bottomRows(stateDimension);

        // Solve P * upper = lower without directly inverting upper.
        Eigen::FullPivLU<Eigen::MatrixXcd> upperSolver(upperEigenvectors.transpose());
        const Eigen::MatrixXcd complexP = upperSolver.solve(lowerEigenvectors.transpose()).transpose();
        if (complexP.imag().norm() > 1e-7 * (1.0 + complexP.real().norm())){
            throw std::runtime_error(
                "CARE solution contains a significant imaginary component.");
        }

        // Numerical eigensolvers can leave a small asymmetry, so explicitly
        // project P back onto the symmetric matrices.
        P_ = complexP.real();
        P_ = 0.5 * (P_ + P_.transpose());
        K_ = rSolver.solve(B_.transpose() * P_);

        const Eigen::MatrixXd residual = A_.transpose() * P_ + P_ * A_ - P_ * B_ * K_ + Q_;
        careResidualNorm_ = residual.norm();

        const double residualScale = 1.0 + (A_.transpose() * P_).norm() + (P_ * A_).norm() + (P_ * B_ * K_).norm() + Q_.norm();
        if (careResidualNorm_ > 1e-7 * residualScale){
            throw std::runtime_error("CARE residual is too large.");
        }
    }
}
