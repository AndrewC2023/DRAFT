#include "QE3DofMissileGainScheduledLQR.hpp"

// Control
#include "Autonomy/Control/LQR.hpp"

// Internal Math
#include "Util/Math/Math.hpp"

// STL
#include <algorithm>
#include <cmath>
#include <limits>
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
    }

    QE3DofMissileGainScheduledLQR::QE3DofMissileGainScheduledLQR(
        Draft::Dynamics::QE3DofMissileParameters parameters,
        Grid grid,
        Weights weights)
        : parameters_(std::move(parameters)),
          grid_(std::move(grid)),
          weights_(weights)
    {
        BuildSchedule();
    }

    Eigen::VectorXd QE3DofMissileGainScheduledLQR::ComputeCommand(
        const Eigen::VectorXd& state,
        const Eigen::Vector3d& referenceOutput) const
    {
        const double altitude = -state(DownPositionIndex);
        const double mach =
            referenceOutput(AirspeedIndex) / CalculateSpeedOfSound(altitude);
        const double mass = state(MassIndex);

        const SchedulePoint& point =
            FindNearestPoint(
                mach,
                altitude,
                mass,
                referenceOutput(FlightPathAngleIndex));

        Eigen::Vector3d reducedState = state.head(3);
        reducedState(HeadingIndex) =
            referenceOutput(HeadingIndex)
            + Draft::Util::Math::WrapAngleToPi(
                reducedState(HeadingIndex) - referenceOutput(HeadingIndex));

        const Eigen::Vector3d error = reducedState - referenceOutput;
        Eigen::Vector3d command = point.trimInput - point.gain * error;

        command(0) = Draft::Util::Math::Saturate(
            command(0), 0.0, parameters_.maxThrust);
        command(1) = Draft::Util::Math::Saturate(
            command(1), -parameters_.maxTurnAngle, parameters_.maxTurnAngle);
        command(2) = Draft::Util::Math::Saturate(
            command(2), -parameters_.maxDiveAngle, parameters_.maxDiveAngle);

        return command;
    }

    std::size_t QE3DofMissileGainScheduledLQR::GetScheduleSize()
        const noexcept
    {
        return schedule_.size();
    }

    void QE3DofMissileGainScheduledLQR::BuildSchedule()
    {
        if (grid_.mach.empty()
            || grid_.altitude.empty()
            || grid_.mass.empty()
            || grid_.flightPathAngle.empty()){
            throw std::invalid_argument("Gain-scheduled LQR grid is empty.");
        }

        for (const double mach : grid_.mach){
            for (const double altitude : grid_.altitude){
                for (const double mass : grid_.mass){
                    for (const double flightPathAngle
                        : grid_.flightPathAngle){
                        schedule_.push_back(
                            MakeSchedulePoint(
                                mach,
                                altitude,
                                mass,
                                flightPathAngle));
                    }
                }
            }
        }
    }

    QE3DofMissileGainScheduledLQR::SchedulePoint
    QE3DofMissileGainScheduledLQR::MakeSchedulePoint(
        double mach,
        double altitude,
        double mass,
        double flightPathAngle) const
    {
        Eigen::Vector3d trimState;
        trimState <<
            MachToAirspeed(mach, altitude),
            0.0,
            flightPathAngle;

        const Eigen::Vector3d trimInput =
            CalculateTrimLikeInput(trimState, mass, altitude);

        Eigen::Matrix3d A;
        Eigen::Matrix3d B;
        LinearizeReducedModel(trimState, trimInput, mass, altitude, A, B);

        Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
        Q(0, 0) = 1.0 / std::pow(weights_.airspeedError, 2);
        Q(1, 1) = 1.0 / std::pow(weights_.headingError, 2);
        Q(2, 2) = 1.0 / std::pow(weights_.flightPathError, 2);

        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        R(0, 0) = 1.0 / std::pow(weights_.thrustCommand, 2);
        R(1, 1) = 1.0 / std::pow(weights_.turnCommand, 2);
        R(2, 2) = 1.0 / std::pow(weights_.diveCommand, 2);

        LQR lqr(A, B, Q, R);

        return {
            .mach = mach,
            .altitude = altitude,
            .mass = mass,
            .flightPathAngle = flightPathAngle,
            .reducedState = trimState,
            .trimInput = trimInput,
            .gain = lqr.GetGain()
        };
    }

    const QE3DofMissileGainScheduledLQR::SchedulePoint&
    QE3DofMissileGainScheduledLQR::FindNearestPoint(
        double mach,
        double altitude,
        double mass,
        double flightPathAngle) const
    {
        const double machScale =
            std::max(grid_.mach.back() - grid_.mach.front(), 1.0);
        const double altitudeScale =
            std::max(grid_.altitude.back() - grid_.altitude.front(), 1.0);
        const double massScale =
            std::max(grid_.mass.back() - grid_.mass.front(), 1.0);
        const double gammaScale =
            std::max(
                grid_.flightPathAngle.back() - grid_.flightPathAngle.front(),
                Draft::Util::Math::Degrees2Radians(1.0));

        const SchedulePoint* nearestPoint = &schedule_.front();
        double bestDistance = std::numeric_limits<double>::infinity();

        for (const SchedulePoint& point : schedule_){
            const double machError = (mach - point.mach) / machScale;
            const double altitudeError =
                (altitude - point.altitude) / altitudeScale;
            const double massError = (mass - point.mass) / massScale;
            const double gammaError =
                (flightPathAngle - point.flightPathAngle) / gammaScale;

            const double distance =
                machError * machError
                + altitudeError * altitudeError
                + massError * massError
                + gammaError * gammaError;

            if (distance < bestDistance){
                bestDistance = distance;
                nearestPoint = &point;
            }
        }

        return *nearestPoint;
    }

    Eigen::Vector3d QE3DofMissileGainScheduledLQR::CalculateTrimLikeInput(
        const Eigen::Vector3d& output,
        double mass,
        double altitude) const
    {
        const double airspeed = output(AirspeedIndex);
        const double flightPathAngle = output(FlightPathAngleIndex);
        const double gravity = CalculateGravity(altitude);
        const double drag = CalculateDrag(airspeed, altitude);

        // This is exact only when thrust-vectoring can satisfy both axial and
        // flight-path balance. Otherwise it gives the nearest saturated
        // feedforward command for this simple no-lift model.
        const double axialTrimThrust =
            drag + mass * gravity * std::sin(flightPathAngle);
        const double gammaHoldThrust =
            std::abs(mass * gravity * std::cos(flightPathAngle))
            / std::max(std::sin(parameters_.maxDiveAngle), 1e-3);
        const double thrust = Draft::Util::Math::Saturate(
            std::max(axialTrimThrust, gammaHoldThrust),
            1.0,
            parameters_.maxThrust);
        const double sineDive = Draft::Util::Math::Saturate(
            mass * gravity * std::cos(flightPathAngle) / thrust,
            -std::sin(parameters_.maxDiveAngle),
            std::sin(parameters_.maxDiveAngle));

        Eigen::Vector3d input;
        input <<
            thrust,
            0.0,
            Draft::Util::Math::Saturate(
                -std::asin(sineDive),
                -parameters_.maxDiveAngle,
                parameters_.maxDiveAngle);
        return input;
    }

    Eigen::Vector3d QE3DofMissileGainScheduledLQR::ReducedDerivative(
        const Eigen::Vector3d& state,
        const Eigen::Vector3d& input,
        double mass,
        double altitude) const
    {
        const double airspeed = state(AirspeedIndex);
        const double flightPathAngle = state(FlightPathAngleIndex);
        const double thrust = input(0);
        const double turnAngle = input(1);
        const double diveAngle = input(2);
        const double gravity = CalculateGravity(altitude);
        const double drag = CalculateDrag(airspeed, altitude);

        Eigen::Vector3d derivative;
        derivative(0) =
            (thrust - drag) / mass
            - gravity * std::sin(flightPathAngle);
        derivative(1) =
            thrust * std::sin(turnAngle)
            / (mass * airspeed * std::cos(flightPathAngle));
        derivative(2) =
            -thrust * std::sin(diveAngle) / (mass * airspeed)
            - gravity * std::cos(flightPathAngle) / airspeed;
        return derivative;
    }

    void QE3DofMissileGainScheduledLQR::LinearizeReducedModel(
        const Eigen::Vector3d& trimState,
        const Eigen::Vector3d& trimInput,
        double mass,
        double altitude,
        Eigen::Matrix3d& A,
        Eigen::Matrix3d& B) const
    {
        A = Eigen::Matrix3d::Zero();
        B = Eigen::Matrix3d::Zero();

        for (int ii = 0; ii < 3; ii++){
            const double perturbation =
                1e-5 * std::max(std::abs(trimState(ii)), 1.0);
            Eigen::Vector3d statePlus = trimState;
            Eigen::Vector3d stateMinus = trimState;
            statePlus(ii) += perturbation;
            stateMinus(ii) -= perturbation;

            A.col(ii) = (
                ReducedDerivative(statePlus, trimInput, mass, altitude)
                - ReducedDerivative(stateMinus, trimInput, mass, altitude)
                ) / (2.0 * perturbation);
        }

        for (int ii = 0; ii < 3; ii++){
            const double perturbation =
                1e-5 * std::max(std::abs(trimInput(ii)), 1.0);
            Eigen::Vector3d inputPlus = trimInput;
            Eigen::Vector3d inputMinus = trimInput;
            inputPlus(ii) += perturbation;
            inputMinus(ii) -= perturbation;

            B.col(ii) = (
                ReducedDerivative(trimState, inputPlus, mass, altitude)
                - ReducedDerivative(trimState, inputMinus, mass, altitude)
                ) / (2.0 * perturbation);
        }
    }

    double QE3DofMissileGainScheduledLQR::CalculateTemperature(
        double altitude) const
    {
        if (altitude < 11000.0){
            return 288.19 - 0.00649 * altitude;
        }
        if (altitude < 25000.0){
            return 216.69;
        }
        return 141.94 + 0.00299 * altitude;
    }

    double QE3DofMissileGainScheduledLQR::CalculateSpeedOfSound(
        double altitude) const
    {
        return std::sqrt(
            parameters_.specificHeatRatio
            * parameters_.idealGasConstant
            * CalculateTemperature(altitude));
    }

    double QE3DofMissileGainScheduledLQR::CalculateGravity(
        double altitude) const
    {
        const double radiusRatio =
            parameters_.earthRadius / (parameters_.earthRadius + altitude);
        return parameters_.standardGravity * radiusRatio * radiusRatio;
    }

    double QE3DofMissileGainScheduledLQR::CalculateDragCoefficient(
        double mach) const
    {
        constexpr double LowerTransonicMach = 0.90;
        constexpr double UpperTransonicMach = 1.10;

        const double lowerCoefficient =
            parameters_.subsonicDragConstant
            / std::sqrt(1.0 - LowerTransonicMach * LowerTransonicMach);
        const double upperCoefficient =
            parameters_.supersonicDragConstant
            / std::sqrt(UpperTransonicMach * UpperTransonicMach - 1.0);

        if (mach < LowerTransonicMach){
            return parameters_.subsonicDragConstant
                / std::sqrt(1.0 - mach * mach);
        }
        if (mach > UpperTransonicMach){
            return parameters_.supersonicDragConstant
                / std::sqrt(mach * mach - 1.0);
        }

        const double interpolationFraction =
            (mach - LowerTransonicMach)
            / (UpperTransonicMach - LowerTransonicMach);
        return lowerCoefficient
            + interpolationFraction
                * (upperCoefficient - lowerCoefficient);
    }

    double QE3DofMissileGainScheduledLQR::CalculateDrag(
        double airspeed,
        double altitude) const
    {
        const double speedOfSound = CalculateSpeedOfSound(altitude);
        const double mach = airspeed / speedOfSound;
        const double density =
            parameters_.seaLevelDensity
            * std::exp(-altitude / parameters_.referenceHeight);
        const double dragCoefficient = CalculateDragCoefficient(mach);

        return 0.5
            * parameters_.referenceArea
            * density
            * airspeed * airspeed
            * dragCoefficient;
    }

    double QE3DofMissileGainScheduledLQR::MachToAirspeed(
        double mach,
        double altitude) const
    {
        return mach * CalculateSpeedOfSound(altitude);
    }
}
