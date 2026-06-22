/*
 * Author: Andrew Campbell
 * Date: 06-19-2026
 */

#ifndef QE3DOFMISSILE_H
#define QE3DOFMISSILE_H

// Base Class
#include "../NonLinearDynamicsBase.hpp"

// STL

namespace Draft::Dynamics
{
    /**
     * @brief Physical and actuator parameters for the QE 3-DOF missile model.
     *
     * All values use SI units. Angular values are expressed in radians.
     */
    struct QE3DofMissileParameters
    {
        double earthRadius;              ///< Mean Earth radius in meters.
        double standardGravity;          ///< Standard gravitational acceleration in m/s^2.
        double seaLevelDensity;          ///< Sea-level atmospheric density in kg/m^3.
        double referenceHeight;          ///< Exponential atmosphere scale height in meters.
        double specificHeatRatio;        ///< Air specific-heat ratio.
        double idealGasConstant;         ///< Specific gas constant for air in J/(kg K).

        double referenceArea;            ///< Aerodynamic reference area in m^2.
        double subsonicDragConstant;     ///< Numerator of the subsonic drag model.
        double supersonicDragConstant;   ///< Numerator of the supersonic drag model.

        double specificImpulse;          ///< Propulsion specific impulse in seconds.
        double dryMass;                  ///< Minimum vehicle mass in kilograms.
        double wetMass;                  ///< Maximum vehicle mass in kilograms.

        double thrustFrequency;          ///< Thrust-loop bandwidth in rad/s.
        double turnFrequency;            ///< Turning-angle loop bandwidth in rad/s.
        double diveFrequency;            ///< Diving-angle loop bandwidth in rad/s.

        double maxThrust;                ///< Maximum thrust in newtons.
        double maxTurnAngle;             ///< Maximum absolute turning angle in radians.
        double maxDiveAngle;             ///< Maximum absolute diving angle in radians.
    };

    /**
     * @brief Nonlinear 3-DOF rocket-powered missile dynamics model.
     *
     * Models airspeed, heading, flight-path angle, mass, NED position, achieved
     * thrust, and achieved turn/dive thrust angles. Commands pass through
     * first-order actuator models. Additional inputs represent NED wind.
     */
    class QE3DofMissile final : public NonLinearDynamicsBase
    {
        public:
            static constexpr int StateDimension = 10;       ///< Number of model states.
            static constexpr int InputDimension = 3;        ///< Number of commanded inputs.
            static constexpr int EnvironmentDimension = 3;  ///< Number of NED wind inputs.

            /**
             * @brief Constructs the missile dynamics model.
             *
             * @param parameters Physical, aerodynamic, and actuator parameters.
             */
            explicit QE3DofMissile(QE3DofMissileParameters parameters);

            /**
             * @brief Returns the parameters used by the model.
             *
             * @return Constant reference to the model parameters.
             */
            const QE3DofMissileParameters& GetParameters() const noexcept{
                return parameters_;
            }

        protected:
            /**
             * @brief Evaluates the ten continuous equations of motion.
             *
             * @param time Current simulation time in seconds.
             * @param state Ten-element physical state vector.
             * @param systemInput Commanded thrust, turn angle, and dive angle.
             * @param environmentalInput Empty for zero wind or three NED wind components.
             * @return Ten-element state derivative vector.
             */
            Eigen::VectorXd EvaluateDerivative(
                double time,
                const Eigen::VectorXd& state,
                const Eigen::VectorXd& systemInput,
                const Eigen::VectorXd& environmentalInput
                ) const override;

            /**
             * @brief Applies mass, thrust, and thrust-angle limits.
             *
             * @param state Propagated state to constrain in place.
             */
            void ApplyConstraints(Eigen::VectorXd& state) const override;

        private:
            QE3DofMissileParameters parameters_; ///< Physical model parameters.

            static constexpr int AirspeedIndex = 0;
            static constexpr int HeadingIndex = 1;
            static constexpr int FlightPathAngleIndex = 2;
            static constexpr int MassIndex = 3;
            static constexpr int NorthPositionIndex = 4;
            static constexpr int EastPositionIndex = 5;
            static constexpr int DownPositionIndex = 6;
            static constexpr int ThrustIndex = 7;
            static constexpr int TurnAngleIndex = 8;
            static constexpr int DiveAngleIndex = 9;

            static constexpr int CommandedThrustIndex = 0;
            static constexpr int CommandedTurnAngleIndex = 1;
            static constexpr int CommandedDiveAngleIndex = 2;

            static constexpr int NorthWindIndex = 0;
            static constexpr int EastWindIndex = 1;
            static constexpr int DownWindIndex = 2;

            /**
             * @brief Validates state, command, and environmental-input dimensions.
             *
             * @throws std::invalid_argument If dimensions differ from the model contract.
             */
            void ValidateDimensions(const Eigen::VectorXd& state, const Eigen::VectorXd& systemInput, const Eigen::VectorXd& environmentalInput) const;

            /** @brief Calculates atmospheric temperature in kelvin. */
            double CalculateTemperature(double altitude) const;

            /** @brief Calculates the local speed of sound in m/s. */
            double CalculateSpeedOfSound(double altitude) const;

            /** @brief Calculates atmospheric density in kg/m^3. */
            double CalculateAtmosphericDensity(double altitude) const;

            /** @brief Calculates Mach number from airspeed and altitude. */
            double CalculateMachNumber(double airspeed, double altitude) const;

            /** @brief Calculates drag coefficient, including transonic interpolation. */
            double CalculateDragCoefficient(double machNumber) const;

            /** @brief Calculates aerodynamic drag force in newtons. */
            double CalculateDragForce(double airspeed, double altitude) const;

            /** @brief Calculates altitude-dependent gravity in m/s^2. */
            double CalculateGravity(double altitude) const;

        
    };
}

#endif // QE3DOFMISSILE_H
