/*
 * Author: Andrew Campbell
 * Date: 06-22-2026
 */

#ifndef QE3DOFMISSILENDI_H
#define QE3DOFMISSILENDI_H

// Dynamics
#include "Dynamics/NonLinearModels/QE3DofMissile.hpp"

// Eigen
#include <Eigen/Dense>

namespace Draft::Autonomy::Control
{
    /**
     * @brief Model-specific nonlinear dynamic inversion controller for the
     * QE 3-DOF missile equations.
     *
     * This controller follows the NDI derivation in the qualifying-exam
     * presentation. The regulated output is
     * @f$y=[v_\infty,\sigma,\gamma]^T@f$ and the command vector is
     * @f$u_c=[T_c,\delta_{t,c},\delta_{d,c}]^T@f$.
     *
     * Commands enter the output dynamics through the first-order actuator
     * states, giving relative degree two in all three output channels:
     *
     * @f[
     * \ddot y = a(x) + B(x)u_c.
     * @f]
     *
     * The controller imposes second-order tracking dynamics through
     *
     * @f[
     * \nu = \ddot y_c + K_d(\dot y_c-\dot y)
     *      + K_p(y_c-y),
     * @f]
     *
     * then solves @f$B(x)u_c=\nu-a(x)@f$ with the triangular form of
     * @f$B(x)@f$.
     */
    class QE3DofMissileNDI
    {
        public:
            /**
             * @brief Diagonal second-order tracking gains.
             *
             * Natural frequencies are in rad/s and damping ratios are
             * unitless. The initial values used in the presentation are
             * `0.30`, `0.20`, and `0.20` rad/s with critical damping.
             */
            struct Gains
            {
                double airspeedNaturalFrequency;
                double headingNaturalFrequency;
                double flightPathNaturalFrequency;

                double airspeedDampingRatio;
                double headingDampingRatio;
                double flightPathDampingRatio;
            };

            /**
             * @brief Constructs the controller from missile parameters and
             * tracking gains.
             *
             * @param parameters Physical and actuator parameters used by the
             * nonlinear missile model.
             * @param gains Desired second-order output tracking gains.
             */
            QE3DofMissileNDI(Draft::Dynamics::QE3DofMissileParameters parameters, Gains gains);

            /**
             * @brief Computes the commanded thrust and thrust-vector angles.
             *
             * @param state Full ten-element missile state.
             * @param commandedOutput Desired `[airspeed, heading,
             * flight-path angle]`.
             * @param commandedOutputRate Desired output rate.
             * @param commandedOutputAcceleration Desired output acceleration.
             * @return Saturated command vector
             * `[T_c, delta_t,c, delta_d,c]`.
             */
            Eigen::VectorXd ComputeCommand(
                const Eigen::VectorXd& state,
                const Eigen::VectorXd& commandedOutput,
                const Eigen::VectorXd& commandedOutputRate,
                const Eigen::VectorXd& commandedOutputAcceleration) const;

        private:
            struct AtmosphereDerivatives
            {
                double drag;
                double dragAirspeedDerivative;
                double dragAltitudeDerivative;
                double gravity;
                double gravityAltitudeDerivative;
            };

            Draft::Dynamics::QE3DofMissileParameters parameters_;
            Gains gains_;

            /** @brief Evaluates output rates using achieved actuator states. */
            Eigen::Vector3d CalculateOutputRate(const Eigen::VectorXd& state, const AtmosphereDerivatives& atmosphere) const;

            /** @brief Evaluates the analytical drift term a(x). */
            Eigen::Vector3d CalculateDrift(const Eigen::VectorXd& state, const AtmosphereDerivatives& atmosphere, const Eigen::Vector3d& outputRate) const;

            /** @brief Evaluates the triangular decoupling matrix B(x). */
            Eigen::Matrix3d CalculateDecouplingMatrix(const Eigen::VectorXd& state) const;

            /** @brief Evaluates drag, gravity, and their needed derivatives. */
            AtmosphereDerivatives CalculateAtmosphereDerivatives(double airspeed, double altitude) const;

            double CalculateTemperature(double altitude) const;
            double CalculateTemperatureDerivative(double altitude) const;
            double CalculateDragCoefficient(double machNumber) const;
            double CalculateDragCoefficientDerivative(double machNumber) const;
    };
}

#endif // QE3DOFMISSILENDI_H
