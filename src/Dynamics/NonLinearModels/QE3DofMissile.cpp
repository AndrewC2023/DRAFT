#include "QE3DofMissile.hpp"

// Internal Math
#include "Util/Math/Math.hpp"

// STL
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace Draft::Dynamics
{
    QE3DofMissile::QE3DofMissile(QE3DofMissileParameters parameters)
        : parameters_(std::move(parameters)) {}

    Eigen::VectorXd QE3DofMissile::EvaluateDerivative(
        [[maybe_unused]] double time,
        const Eigen::VectorXd& state,
        const Eigen::VectorXd& systemInput,
        const Eigen::VectorXd& environmentalInput) const
    {
        ValidateDimensions(state, systemInput, environmentalInput);

        const double airspeed = state(AirspeedIndex);
        const double heading = state(HeadingIndex);
        const double flightPathAngle = state(FlightPathAngleIndex);
        const double mass = state(MassIndex);
        const double downPosition = state(DownPositionIndex);
        const double thrust = state(ThrustIndex);
        const double turnAngle = state(TurnAngleIndex);
        const double diveAngle = state(DiveAngleIndex);

        if (airspeed <= 0.0){
            throw std::domain_error("Airspeed must be positive.");
        }
        if (mass <= 0.0){
            throw std::domain_error("Mass must be positive.");
        }
        if (std::abs(std::cos(flightPathAngle)) <= 1e-10){
            throw std::domain_error("Flight-path angle produces a singular heading rate.");
        }

        const double altitude = -downPosition;
        const double gravity = CalculateGravity(altitude);
        const double drag = CalculateDragForce(airspeed, altitude);

        const double commandedThrust = Draft::Util::Math::Saturate(systemInput(CommandedThrustIndex), 0.0, parameters_.maxThrust);
        const double commandedTurnAngle = Draft::Util::Math::Saturate(systemInput(CommandedTurnAngleIndex), -parameters_.maxTurnAngle, parameters_.maxTurnAngle);
        const double commandedDiveAngle = Draft::Util::Math::Saturate(systemInput(CommandedDiveAngleIndex), -parameters_.maxDiveAngle, parameters_.maxDiveAngle);

        // check to include wind or not
        double northWind = 0.0;
        double eastWind = 0.0;
        double downWind = 0.0;
        if (environmentalInput.size() == EnvironmentDimension){
            northWind = environmentalInput(NorthWindIndex);
            eastWind = environmentalInput(EastWindIndex);
            downWind = environmentalInput(DownWindIndex);
        }

        Eigen::VectorXd derivative = Eigen::VectorXd::Zero(StateDimension);
        derivative(AirspeedIndex) = (thrust - drag) / mass - std::sin(flightPathAngle) * gravity;
        derivative(HeadingIndex) = thrust * std::sin(turnAngle) / (mass * airspeed * std::cos(flightPathAngle));
        derivative(FlightPathAngleIndex) = -thrust * std::sin(diveAngle) / (mass * airspeed) - std::cos(flightPathAngle) * gravity / airspeed;
        derivative(MassIndex) = mass > parameters_.dryMass ? -thrust / (parameters_.specificImpulse * parameters_.standardGravity) : 0.0;
        derivative(NorthPositionIndex) = airspeed * std::cos(flightPathAngle) * std::cos(heading) + northWind;
        derivative(EastPositionIndex) = airspeed * std::cos(flightPathAngle) * std::sin(heading) + eastWind;
        derivative(DownPositionIndex) = -airspeed * std::sin(flightPathAngle) + downWind;
        derivative(ThrustIndex) = parameters_.thrustFrequency * (commandedThrust - thrust);
        derivative(TurnAngleIndex) = parameters_.turnFrequency * (commandedTurnAngle - turnAngle);
        derivative(DiveAngleIndex) = parameters_.diveFrequency * (commandedDiveAngle - diveAngle);

        return derivative;
    }

    void QE3DofMissile::ApplyConstraints(Eigen::VectorXd& state) const
    {
        Draft::Util::Math::ValidateVectorDimension(state, StateDimension, "QE3DofMissile state");
        Draft::Util::Math::ValidateFinite(state, "QE3DofMissile state");

        state(MassIndex) = Draft::Util::Math::Saturate(state(MassIndex), parameters_.dryMass, parameters_.wetMass);
        state(ThrustIndex) = Draft::Util::Math::Saturate(state(ThrustIndex), 0.0, parameters_.maxThrust);
        state(TurnAngleIndex) = Draft::Util::Math::Saturate(state(TurnAngleIndex), -parameters_.maxTurnAngle, parameters_.maxTurnAngle);
        state(DiveAngleIndex) = Draft::Util::Math::Saturate(state(DiveAngleIndex), -parameters_.maxDiveAngle, parameters_.maxDiveAngle);
    }

    void QE3DofMissile::ValidateDimensions(
        const Eigen::VectorXd& state,
        const Eigen::VectorXd& systemInput,
        const Eigen::VectorXd& environmentalInput) const
    {
        Draft::Util::Math::ValidateVectorDimension(state, StateDimension, "QE3DofMissile state");
        Draft::Util::Math::ValidateVectorDimension(systemInput, InputDimension, "QE3DofMissile system input");

        if (environmentalInput.size() != 0 && environmentalInput.size() != EnvironmentDimension){
            throw std::invalid_argument("QE3DofMissile environmental input must be empty or have three elements.");
        }

        Draft::Util::Math::ValidateFinite(state, "QE3DofMissile state");
        Draft::Util::Math::ValidateFinite(systemInput, "QE3DofMissile system input");
        Draft::Util::Math::ValidateFinite(environmentalInput, "QE3DofMissile environmental input");
    }

    double QE3DofMissile::CalculateTemperature(double altitude) const
    {
        if (altitude < 11000.0){
            return 288.19 - 0.00649 * altitude;
        } else if (altitude < 25000.0){
            return 216.69;
        } else return 141.94 + 0.00299 * altitude;
    }

    double QE3DofMissile::CalculateSpeedOfSound(double altitude) const
    {
        const double temperature = CalculateTemperature(altitude);
        if (temperature <= 0.0){
            throw std::domain_error("Atmospheric temperature must be positive.");
        }

        return std::sqrt(parameters_.specificHeatRatio * parameters_.idealGasConstant * temperature);
    }

    double QE3DofMissile::CalculateAtmosphericDensity(double altitude) const
    {
        return parameters_.seaLevelDensity * std::exp(-altitude / parameters_.referenceHeight);
    }

    double QE3DofMissile::CalculateMachNumber(double airspeed, double altitude) const
    {
        return std::abs(airspeed) / CalculateSpeedOfSound(altitude);
    }

    double QE3DofMissile::CalculateDragCoefficient(double machNumber) const
    {
        // setup bounds for M = 1 asyptote and handling for the transonic region
        constexpr double LowerTransonicMach = 0.90;
        constexpr double UpperTransonicMach = 1.1;

        if (machNumber < 0.0){
            throw std::invalid_argument("Mach number must not be negative.");
        }

        const double lowerCoefficient = parameters_.subsonicDragConstant / std::sqrt(1.0 - LowerTransonicMach * LowerTransonicMach);
        const double upperCoefficient = parameters_.supersonicDragConstant / std::sqrt(UpperTransonicMach * UpperTransonicMach - 1.0);

        if (machNumber < LowerTransonicMach){
            return parameters_.subsonicDragConstant / std::sqrt(1.0 - machNumber * machNumber);
        }
        if (machNumber > UpperTransonicMach){
            return parameters_.supersonicDragConstant / std::sqrt(machNumber * machNumber - 1.0);
        }

        const double interpolationFraction = (machNumber - LowerTransonicMach) / (UpperTransonicMach - LowerTransonicMach);

        return lowerCoefficient + interpolationFraction * (upperCoefficient - lowerCoefficient);
    }

    double QE3DofMissile::CalculateDragForce(double airspeed, double altitude) const
    {
        const double density = CalculateAtmosphericDensity(altitude);
        const double machNumber = CalculateMachNumber(airspeed, altitude);
        const double dragCoefficient = CalculateDragCoefficient(machNumber);

        return 0.5 * parameters_.referenceArea * density * airspeed * airspeed * dragCoefficient;
    }

    double QE3DofMissile::CalculateGravity(double altitude) const
    {
        const double radius = parameters_.earthRadius + altitude;
        if (radius <= 0.0){
            throw std::domain_error("Altitude produces a non-positive distance from the Earth center.");
        }

        const double radiusRatio = parameters_.earthRadius / radius;
        return parameters_.standardGravity * radiusRatio * radiusRatio;
    }
}
