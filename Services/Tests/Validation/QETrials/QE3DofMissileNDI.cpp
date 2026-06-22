#include "QE3DofMissileNDI.hpp"

// Internal Math
#include "Util/Math/Math.hpp"

// STL
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace Draft::Autonomy::Control
{
    namespace
    {
        constexpr int AirspeedIndex = 0;
        constexpr int HeadingIndex = 1;
        constexpr int FlightPathAngleIndex = 2;
        constexpr int MassIndex = 3;
        constexpr int DownPositionIndex = 6;
        constexpr int ThrustIndex = 7;
        constexpr int TurnAngleIndex = 8;
        constexpr int DiveAngleIndex = 9;
    }

    QE3DofMissileNDI::QE3DofMissileNDI(
        Draft::Dynamics::QE3DofMissileParameters parameters,
        Gains gains)
        : parameters_(std::move(parameters)),
          gains_(gains)
    {
    }

    Eigen::VectorXd QE3DofMissileNDI::ComputeCommand(
        const Eigen::VectorXd& state,
        const Eigen::VectorXd& commandedOutput,
        const Eigen::VectorXd& commandedOutputRate,
        const Eigen::VectorXd& commandedOutputAcceleration) const
    {
        const double airspeed = state(AirspeedIndex);
        const double altitude = -state(DownPositionIndex);

        const AtmosphereDerivatives atmosphere =
            CalculateAtmosphereDerivatives(airspeed, altitude);

        const Eigen::Vector3d outputRate =
            CalculateOutputRate(state, atmosphere);
        const Eigen::Vector3d drift =
            CalculateDrift(state, atmosphere, outputRate);
        const Eigen::Matrix3d decoupling =
            CalculateDecouplingMatrix(state);

        Eigen::Vector3d error;
        error <<
            commandedOutput(0) - state(AirspeedIndex),
            Draft::Util::Math::WrapAngleToPi(
                commandedOutput(1) - state(HeadingIndex)),
            commandedOutput(2) - state(FlightPathAngleIndex);

        const Eigen::Vector3d errorRate =
            commandedOutputRate - outputRate;

        Eigen::Vector3d proportionalGain;
        proportionalGain <<
            gains_.airspeedNaturalFrequency
                * gains_.airspeedNaturalFrequency,
            gains_.headingNaturalFrequency
                * gains_.headingNaturalFrequency,
            gains_.flightPathNaturalFrequency
                * gains_.flightPathNaturalFrequency;

        Eigen::Vector3d derivativeGain;
        derivativeGain <<
            2.0 * gains_.airspeedDampingRatio
                * gains_.airspeedNaturalFrequency,
            2.0 * gains_.headingDampingRatio
                * gains_.headingNaturalFrequency,
            2.0 * gains_.flightPathDampingRatio
                * gains_.flightPathNaturalFrequency;

        const Eigen::Vector3d virtualControl =
            commandedOutputAcceleration
            + derivativeGain.cwiseProduct(errorRate)
            + proportionalGain.cwiseProduct(error);

        // The decoupling matrix is triangular, so this matches the
        // presentation's sequential solve without forming an explicit inverse.
        Eigen::Vector3d command;
        command(0) = (virtualControl(0) - drift(0)) / decoupling(0, 0);
        command(1) = (
            virtualControl(1)
            - drift(1)
            - decoupling(1, 0) * command(0)
            ) / decoupling(1, 1);
        command(2) = (
            virtualControl(2)
            - drift(2)
            - decoupling(2, 0) * command(0)
            ) / decoupling(2, 2);

        command(0) = Draft::Util::Math::Saturate(
            command(0), 0.0, parameters_.maxThrust);
        command(1) = Draft::Util::Math::Saturate(
            command(1), -parameters_.maxTurnAngle, parameters_.maxTurnAngle);
        command(2) = Draft::Util::Math::Saturate(
            command(2), -parameters_.maxDiveAngle, parameters_.maxDiveAngle);

        return command;
    }

    Eigen::Vector3d QE3DofMissileNDI::CalculateOutputRate(
        const Eigen::VectorXd& state,
        const AtmosphereDerivatives& atmosphere) const
    {
        const double airspeed = state(AirspeedIndex);
        const double flightPathAngle = state(FlightPathAngleIndex);
        const double mass = state(MassIndex);
        const double thrust = state(ThrustIndex);
        const double turnAngle = state(TurnAngleIndex);
        const double diveAngle = state(DiveAngleIndex);

        Eigen::Vector3d outputRate;
        outputRate(0) =
            (thrust - atmosphere.drag) / mass
            - atmosphere.gravity * std::sin(flightPathAngle);
        outputRate(1) =
            thrust * std::sin(turnAngle)
            / (mass * airspeed * std::cos(flightPathAngle));
        outputRate(2) =
            -thrust * std::sin(diveAngle) / (mass * airspeed)
            - atmosphere.gravity * std::cos(flightPathAngle) / airspeed;

        return outputRate;
    }

    Eigen::Vector3d QE3DofMissileNDI::CalculateDrift(
        const Eigen::VectorXd& state,
        const AtmosphereDerivatives& atmosphere,
        const Eigen::Vector3d& outputRate) const
    {
        const double airspeed = state(AirspeedIndex);
        const double flightPathAngle = state(FlightPathAngleIndex);
        const double mass = state(MassIndex);
        const double thrust = state(ThrustIndex);
        const double turnAngle = state(TurnAngleIndex);
        const double diveAngle = state(DiveAngleIndex);

        const double airspeedRate = outputRate(0);
        const double flightPathRate = outputRate(2);
        const double massRate = mass > parameters_.dryMass
            ? -thrust / (parameters_.specificImpulse
                * parameters_.standardGravity)
            : 0.0;

        const double altitudeRate =
            airspeed * std::sin(flightPathAngle);
        const double dragRate =
            atmosphere.dragAirspeedDerivative * airspeedRate
            + atmosphere.dragAltitudeDerivative * altitudeRate;
        const double gravityRate =
            atmosphere.gravityAltitudeDerivative * altitudeRate;

        const double cosFlightPath = std::cos(flightPathAngle);
        const double sinFlightPath = std::sin(flightPathAngle);
        const double sinTurn = std::sin(turnAngle);
        const double cosTurn = std::cos(turnAngle);
        const double sinDive = std::sin(diveAngle);
        const double cosDive = std::cos(diveAngle);

        Eigen::Vector3d drift;

        drift(0) =
            (-parameters_.thrustFrequency * thrust - dragRate) / mass
            - ((thrust - atmosphere.drag) * massRate) / (mass * mass)
            - gravityRate * sinFlightPath
            - atmosphere.gravity * cosFlightPath * flightPathRate;

        drift(1) =
            (sinTurn / (mass * airspeed * cosFlightPath))
                * (-parameters_.thrustFrequency * thrust)
            + (thrust * cosTurn / (mass * airspeed * cosFlightPath))
                * (-parameters_.turnFrequency * turnAngle)
            - (thrust * sinTurn * massRate)
                / (mass * mass * airspeed * cosFlightPath)
            - (thrust * sinTurn * airspeedRate)
                / (mass * airspeed * airspeed * cosFlightPath)
            + (thrust * sinTurn * sinFlightPath * flightPathRate)
                / (mass * airspeed * cosFlightPath * cosFlightPath);

        drift(2) =
            parameters_.thrustFrequency * thrust * sinDive
                / (mass * airspeed)
            + parameters_.diveFrequency * thrust * diveAngle * cosDive
                / (mass * airspeed)
            + thrust * sinDive * massRate
                / (mass * mass * airspeed)
            + thrust * sinDive * airspeedRate
                / (mass * airspeed * airspeed)
            - gravityRate * cosFlightPath / airspeed
            + atmosphere.gravity * sinFlightPath * flightPathRate / airspeed
            + atmosphere.gravity * cosFlightPath * airspeedRate
                / (airspeed * airspeed);

        return drift;
    }

    Eigen::Matrix3d QE3DofMissileNDI::CalculateDecouplingMatrix(
        const Eigen::VectorXd& state) const
    {
        const double airspeed = state(AirspeedIndex);
        const double flightPathAngle = state(FlightPathAngleIndex);
        const double mass = state(MassIndex);
        const double thrust = state(ThrustIndex);
        const double turnAngle = state(TurnAngleIndex);
        const double diveAngle = state(DiveAngleIndex);

        const double cosFlightPath = std::cos(flightPathAngle);
        const double cosTurn = std::cos(turnAngle);
        const double sinTurn = std::sin(turnAngle);
        const double cosDive = std::cos(diveAngle);
        const double sinDive = std::sin(diveAngle);

        if (airspeed <= 0.0 || mass <= 0.0 || thrust <= 0.0){
            throw std::domain_error("NDI requires positive airspeed, mass, and thrust.");
        }

        Eigen::Matrix3d decoupling = Eigen::Matrix3d::Zero();
        decoupling(0, 0) = parameters_.thrustFrequency / mass;
        decoupling(1, 0) =
            parameters_.thrustFrequency * sinTurn
            / (mass * airspeed * cosFlightPath);
        decoupling(1, 1) =
            parameters_.turnFrequency * thrust * cosTurn
            / (mass * airspeed * cosFlightPath);
        decoupling(2, 0) =
            -parameters_.thrustFrequency * sinDive
            / (mass * airspeed);
        decoupling(2, 2) =
            -parameters_.diveFrequency * thrust * cosDive
            / (mass * airspeed);

        return decoupling;
    }

    QE3DofMissileNDI::AtmosphereDerivatives
    QE3DofMissileNDI::CalculateAtmosphereDerivatives(
        double airspeed,
        double altitude) const
    {
        const double temperature = CalculateTemperature(altitude);
        const double temperatureDerivative =
            CalculateTemperatureDerivative(altitude);
        const double speedOfSound = std::sqrt(
            parameters_.specificHeatRatio
            * parameters_.idealGasConstant
            * temperature);
        const double speedOfSoundAltitudeDerivative =
            0.5 * speedOfSound * temperatureDerivative / temperature;

        const double machNumber = airspeed / speedOfSound;
        const double machAirspeedDerivative = 1.0 / speedOfSound;
        const double machAltitudeDerivative =
            -airspeed
            * speedOfSoundAltitudeDerivative
            / (speedOfSound * speedOfSound);

        const double density =
            parameters_.seaLevelDensity
            * std::exp(-altitude / parameters_.referenceHeight);
        const double densityAltitudeDerivative =
            -density / parameters_.referenceHeight;

        const double dragCoefficient =
            CalculateDragCoefficient(machNumber);
        const double dragCoefficientDerivative =
            CalculateDragCoefficientDerivative(machNumber);

        const double drag =
            0.5
            * parameters_.referenceArea
            * density
            * airspeed * airspeed
            * dragCoefficient;

        const double dragAirspeedDerivative =
            0.5
            * parameters_.referenceArea
            * density
            * (
                2.0 * airspeed * dragCoefficient
                + airspeed * airspeed
                    * dragCoefficientDerivative
                    * machAirspeedDerivative
            );

        const double dragAltitudeDerivative =
            0.5
            * parameters_.referenceArea
            * airspeed * airspeed
            * (
                densityAltitudeDerivative * dragCoefficient
                + density
                    * dragCoefficientDerivative
                    * machAltitudeDerivative
            );

        const double radius = parameters_.earthRadius + altitude;
        const double radiusRatio = parameters_.earthRadius / radius;
        const double gravity =
            parameters_.standardGravity * radiusRatio * radiusRatio;
        const double gravityAltitudeDerivative =
            -2.0 * gravity / radius;

        return {
            .drag = drag,
            .dragAirspeedDerivative = dragAirspeedDerivative,
            .dragAltitudeDerivative = dragAltitudeDerivative,
            .gravity = gravity,
            .gravityAltitudeDerivative = gravityAltitudeDerivative
        };
    }

    double QE3DofMissileNDI::CalculateTemperature(double altitude) const
    {
        if (altitude < 11000.0){
            return 288.19 - 0.00649 * altitude;
        }
        if (altitude < 25000.0){
            return 216.69;
        }
        return 141.94 + 0.00299 * altitude;
    }

    double QE3DofMissileNDI::CalculateTemperatureDerivative(
        double altitude) const
    {
        if (altitude < 11000.0){
            return -0.00649;
        }
        if (altitude < 25000.0){
            return 0.0;
        }
        return 0.00299;
    }

    double QE3DofMissileNDI::CalculateDragCoefficient(
        double machNumber) const
    {
        constexpr double LowerTransonicMach = 0.90;
        constexpr double UpperTransonicMach = 1.10;

        const double lowerCoefficient =
            parameters_.subsonicDragConstant
            / std::sqrt(1.0 - LowerTransonicMach * LowerTransonicMach);
        const double upperCoefficient =
            parameters_.supersonicDragConstant
            / std::sqrt(UpperTransonicMach * UpperTransonicMach - 1.0);

        if (machNumber < LowerTransonicMach){
            return parameters_.subsonicDragConstant
                / std::sqrt(1.0 - machNumber * machNumber);
        }
        if (machNumber > UpperTransonicMach){
            return parameters_.supersonicDragConstant
                / std::sqrt(machNumber * machNumber - 1.0);
        }

        const double interpolationFraction =
            (machNumber - LowerTransonicMach)
            / (UpperTransonicMach - LowerTransonicMach);

        return lowerCoefficient
            + interpolationFraction
                * (upperCoefficient - lowerCoefficient);
    }

    double QE3DofMissileNDI::CalculateDragCoefficientDerivative(
        double machNumber) const
    {
        constexpr double LowerTransonicMach = 0.90;
        constexpr double UpperTransonicMach = 1.10;

        const double lowerCoefficient =
            parameters_.subsonicDragConstant
            / std::sqrt(1.0 - LowerTransonicMach * LowerTransonicMach);
        const double upperCoefficient =
            parameters_.supersonicDragConstant
            / std::sqrt(UpperTransonicMach * UpperTransonicMach - 1.0);

        if (machNumber < LowerTransonicMach){
            return parameters_.subsonicDragConstant
                * machNumber
                / std::pow(1.0 - machNumber * machNumber, 1.5);
        }
        if (machNumber > UpperTransonicMach){
            return -parameters_.supersonicDragConstant
                * machNumber
                / std::pow(machNumber * machNumber - 1.0, 1.5);
        }

        return (upperCoefficient - lowerCoefficient)
            / (UpperTransonicMach - LowerTransonicMach);
    }
}
