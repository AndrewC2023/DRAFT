/*
 * Author: Andrew Campbell
 * Date: 06-22-2026
 */

#ifndef QE3DOFMISSILEGAINSCHEDULEDLQR_H
#define QE3DOFMISSILEGAINSCHEDULEDLQR_H

// Dynamics
#include "Dynamics/NonLinearModels/QE3DofMissile.hpp"

// Eigen
#include <Eigen/Dense>

// STL
#include <cstddef>
#include <vector>

namespace Draft::Autonomy::Control
{
    /**
     * @brief Trial-level gain-scheduled LQR for the QE 3-DOF missile model.
     *
     * This class precomputes continuous-time LQR gains on a small flight
     * condition grid. The reduced controlled state is
     * @f$[v_\infty,\sigma,\gamma]^T@f$ and the command is
     * @f$[T_c,\delta_{t,c},\delta_{d,c}]^T@f$.
     *
     * At runtime the nearest schedule point is selected from the commanded
     * Mach number, current altitude, current mass, and commanded flight-path
     * angle. The command law remains the usual local LQR law:
     *
     * @f[
     * u_c = u_0(\rho) - K(\rho)(x_r - x_c),
     * @f]
     *
     * where @f$\rho@f$ is the selected schedule point. The implementation uses
     * nearest-neighbor scheduling rather than interpolation so the behavior is
     * easy to describe in the report.
     */
    class QE3DofMissileGainScheduledLQR
    {
        public:
            /** @brief Scheduling grid values in physical units. */
            struct Grid
            {
                std::vector<double> mach;            ///< Mach grid.
                std::vector<double> altitude;        ///< Altitude grid in meters.
                std::vector<double> mass;            ///< Mass grid in kilograms.
                std::vector<double> flightPathAngle; ///< Gamma grid in radians.
            };

            /** @brief Allowable state errors and input changes used for Q and R. */
            struct Weights
            {
                double airspeedError;        ///< Allowed airspeed error in m/s.
                double headingError;         ///< Allowed heading error in radians.
                double flightPathError;      ///< Allowed gamma error in radians.
                double thrustCommand;        ///< Allowed thrust command in newtons.
                double turnCommand;          ///< Allowed turn command in radians.
                double diveCommand;          ///< Allowed dive command in radians.
            };

            /**
             * @brief Builds the gain schedule.
             *
             * @param parameters Missile model parameters.
             * @param grid Scheduling grid.
             * @param weights LQR state and input weights.
             */
            QE3DofMissileGainScheduledLQR(Draft::Dynamics::QE3DofMissileParameters parameters, Grid grid, Weights weights);

            /**
             * @brief Computes a saturated scheduled LQR command.
             *
             * @param state Full ten-state missile state.
             * @param referenceOutput Desired `[airspeed, heading, gamma]`.
             * @return Commanded `[T_c, delta_t,c, delta_d,c]`.
             */
            Eigen::VectorXd ComputeCommand(const Eigen::VectorXd& state, const Eigen::Vector3d& referenceOutput) const;

            /** @brief Returns the number of schedule points that were built. */
            std::size_t GetScheduleSize() const noexcept;

        private:
            struct SchedulePoint
            {
                double mach;
                double altitude;
                double mass;
                double flightPathAngle;
                Eigen::Vector3d reducedState;
                Eigen::Vector3d trimInput;
                Eigen::Matrix3d gain;
            };

            Draft::Dynamics::QE3DofMissileParameters parameters_;
            Grid grid_;
            Weights weights_;
            std::vector<SchedulePoint> schedule_;

            void BuildSchedule();

            SchedulePoint MakeSchedulePoint(double mach, double altitude, double mass, double flightPathAngle) const;

            const SchedulePoint& FindNearestPoint(double mach, double altitude, double mass, double flightPathAngle) const;

            Eigen::Vector3d CalculateTrimLikeInput(const Eigen::Vector3d& output, double mass, double altitude) const;

            Eigen::Vector3d ReducedDerivative(const Eigen::Vector3d& state, const Eigen::Vector3d& input, double mass, double altitude) const;

            void LinearizeReducedModel(
                const Eigen::Vector3d& trimState,
                const Eigen::Vector3d& trimInput,
                double mass,
                double altitude,
                Eigen::Matrix3d& A,
                Eigen::Matrix3d& B) const;

            double CalculateTemperature(double altitude) const;
            double CalculateSpeedOfSound(double altitude) const;
            double CalculateGravity(double altitude) const;
            double CalculateDragCoefficient(double mach) const;
            double CalculateDrag(double airspeed, double altitude) const;
            double MachToAirspeed(double mach, double altitude) const;
    };
}

#endif // QE3DOFMISSILEGAINSCHEDULEDLQR_H
