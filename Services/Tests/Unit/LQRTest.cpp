/*
 * Author: Andrew Campbell
 * Date: 06-22-2026
 */

// Control
#include "Autonomy/Control/LQR.hpp"

// Google Test
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Dense>

// STL
#include <cmath>

TEST(LQRTest, SolvesScalarCare)
{
    Eigen::MatrixXd A(1, 1);
    Eigen::MatrixXd B(1, 1);
    Eigen::MatrixXd Q(1, 1);
    Eigen::MatrixXd R(1, 1);
    A << 0.0;
    B << 1.0;
    Q << 1.0;
    R << 1.0;

    Draft::Autonomy::Control::LQR controller(A, B, Q, R);

    EXPECT_NEAR(controller.GetRiccatiSolution()(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(controller.GetGain()(0, 0), 1.0, 1e-10);
    EXPECT_LT(controller.GetCareResidualNorm(), 1e-10);
}

TEST(LQRTest, SolvesContinuousDoubleIntegrator)
{
    Eigen::MatrixXd A(2, 2);
    Eigen::MatrixXd B(2, 1);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R(1, 1);
    A << 0.0, 1.0,
         0.0, 0.0;
    B << 0.0,
         1.0;
    R << 1.0;

    Draft::Autonomy::Control::LQR controller(A, B, Q, R);

    EXPECT_NEAR(controller.GetGain()(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(controller.GetGain()(0, 1), std::sqrt(3.0), 1e-9);
    EXPECT_LT(controller.GetCareResidualNorm(), 1e-9);
}

TEST(LQRTest, ComputesReferenceTrackingControl)
{
    Eigen::MatrixXd A(1, 1);
    Eigen::MatrixXd B(1, 1);
    Eigen::MatrixXd Q(1, 1);
    Eigen::MatrixXd R(1, 1);
    A << 0.0;
    B << 1.0;
    Q << 1.0;
    R << 1.0;

    Draft::Autonomy::Control::LQR controller(A, B, Q, R);

    Eigen::VectorXd state(1);
    Eigen::VectorXd referenceState(1);
    Eigen::VectorXd referenceInput(1);
    state << 2.0;
    referenceState << 0.0;
    referenceInput << 0.0;

    const Eigen::VectorXd input = controller.ComputeControl(
        state, referenceState, referenceInput);

    EXPECT_NEAR(input(0), -2.0, 1e-10);
}
