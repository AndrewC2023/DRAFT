/*
 * Author: Andrew Campbell
 * Date: 06-21-2026
 */

// Control
#include "Autonomy/Control/LQR.hpp"
#include "QE3DofMissileGainScheduledLQR.hpp"
#include "QE3DofMissileNDI.hpp"

// Dynamics
#include "Dynamics/NonLinearModels/QE3DofMissile.hpp"

// Internal Math
#include "Util/Math/Math.hpp"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

// JSON
#include <nlohmann/json.hpp>

// STL
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    struct ReferenceCommand
    {
        Eigen::Vector3d output;
        Eigen::Vector3d outputRate;
        Eigen::Vector3d outputAcceleration;
    };

    struct Scenario
    {
        std::string name;
        std::string description;

        double altitude;
        double mass;

        double initialMach;
        double finalMach;
        double initialHeading;
        double finalHeading;
        double initialFlightPathAngle;
        double finalFlightPathAngle;

        double finalTime;
        double transitionTime;
        bool runLqr;
    };

    template<typename Derived>
    std::vector<double> ToStdVector(const Eigen::MatrixBase<Derived>& vector)
    {
        std::vector<double> values;
        values.reserve(static_cast<std::size_t>(vector.size()));
        for (Eigen::Index ii = 0; ii < vector.size(); ii++){
            values.push_back(vector(ii));
        }
        return values;
    }

    ReferenceCommand CalculateLinearReference(
        double time,
        double transitionTime,
        const Eigen::Vector3d& initialOutput,
        const Eigen::Vector3d& finalOutput)
    {
        // Linear references make it clear what the controller is being asked
        // to do, and the rate is needed by NDI for dynamic inversion.
        Eigen::Vector3d outputChange = finalOutput - initialOutput;
        outputChange(1) = Draft::Util::Math::WrapAngleToPi(outputChange(1));

        const double fraction = transitionTime > 0.0
            ? std::clamp(time / transitionTime, 0.0, 1.0)
            : 1.0;
        const bool isTransitioning =
            transitionTime > 0.0 && time <= transitionTime;

        ReferenceCommand command;
        command.output = initialOutput + fraction * outputChange;
        command.output(1) =
            Draft::Util::Math::WrapAngleToPi(command.output(1));
        if (isTransitioning){
            command.outputRate = outputChange / transitionTime;
        } else {
            command.outputRate = Eigen::Vector3d::Zero();
        }
        command.outputAcceleration = Eigen::Vector3d::Zero();
        return command;
    }

    double CalculateTemperature(double altitude)
    {
        if (altitude < 11000.0){
            return 288.19 - 0.00649 * altitude;
        }
        if (altitude < 25000.0){
            return 216.69;
        }
        return 141.94 + 0.00299 * altitude;
    }

    double CalculateSpeedOfSound(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        double altitude)
    {
        return std::sqrt(
            parameters.specificHeatRatio
            * parameters.idealGasConstant
            * CalculateTemperature(altitude));
    }

    double CalculateGravity(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        double altitude)
    {
        const double radiusRatio =
            parameters.earthRadius / (parameters.earthRadius + altitude);
        return parameters.standardGravity * radiusRatio * radiusRatio;
    }

    double CalculateDragCoefficient(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        double mach)
    {
        constexpr double LowerTransonicMach = 0.90;
        constexpr double UpperTransonicMach = 1.10;

        const double lowerCoefficient =
            parameters.subsonicDragConstant
            / std::sqrt(1.0 - LowerTransonicMach * LowerTransonicMach);
        const double upperCoefficient =
            parameters.supersonicDragConstant
            / std::sqrt(UpperTransonicMach * UpperTransonicMach - 1.0);

        if (mach < LowerTransonicMach){
            return parameters.subsonicDragConstant
                / std::sqrt(1.0 - mach * mach);
        }
        if (mach > UpperTransonicMach){
            return parameters.supersonicDragConstant
                / std::sqrt(mach * mach - 1.0);
        }

        const double interpolationFraction =
            (mach - LowerTransonicMach)
            / (UpperTransonicMach - LowerTransonicMach);
        return lowerCoefficient
            + interpolationFraction
                * (upperCoefficient - lowerCoefficient);
    }

    double CalculateDrag(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        double airspeed,
        double altitude)
    {
        const double speedOfSound =
            CalculateSpeedOfSound(parameters, altitude);
        const double mach = airspeed / speedOfSound;
        const double density = parameters.seaLevelDensity
            * std::exp(-altitude / parameters.referenceHeight);
        const double dragCoefficient = CalculateDragCoefficient(parameters, mach);

        return 0.5
            * parameters.referenceArea
            * density
            * airspeed * airspeed
            * dragCoefficient;
    }

    double MachToAirspeed(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        double mach,
        double altitude)
    {
        return mach * CalculateSpeedOfSound(parameters, altitude);
    }

    Eigen::Vector3d MakeOutput(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Scenario& scenario,
        bool useFinalOutput)
    {
        Eigen::Vector3d output;
        output <<
            MachToAirspeed(parameters, useFinalOutput ? scenario.finalMach : scenario.initialMach, scenario.altitude),
            useFinalOutput
                ? scenario.finalHeading
                : scenario.initialHeading,
            useFinalOutput
                ? scenario.finalFlightPathAngle
                : scenario.initialFlightPathAngle;
        return output;
    }

    Eigen::VectorXd CalculateTrimLikeInput(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Eigen::Vector3d& output,
        double mass,
        double altitude)
    {
        const double airspeed = output(0);
        const double flightPathAngle = output(2);
        const double gravity = CalculateGravity(parameters, altitude);
        const double drag = CalculateDrag(parameters, airspeed, altitude);

        // This is exact level trim when the model has enough thrust-vector
        // authority. If not, it picks the nearest powered state the simple
        // thrust-only model can represent.
        const double axialTrimThrust =
            drag + mass * gravity * std::sin(flightPathAngle);
        const double gammaHoldThrust =
            std::abs(mass * gravity * std::cos(flightPathAngle))
            / std::max(std::sin(parameters.maxDiveAngle), 1e-3);
        const double thrust = Draft::Util::Math::Saturate(std::max(axialTrimThrust, gammaHoldThrust), 1.0, parameters.maxThrust);
        const double sineDive = Draft::Util::Math::Saturate(
            mass * gravity * std::cos(flightPathAngle) / thrust,
            -std::sin(parameters.maxDiveAngle),
            std::sin(parameters.maxDiveAngle));
        const double diveAngle = -std::asin(sineDive);

        Eigen::VectorXd input(3);
        input <<
            thrust,
            0.0,
            Draft::Util::Math::Saturate(diveAngle, -parameters.maxDiveAngle, parameters.maxDiveAngle);
        return input;
    }

    Eigen::VectorXd MakeInitialState(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Eigen::Vector3d& initialOutput,
        double mass,
        double altitude)
    {
        // Keep the simulation initial actuator states close to a local
        // force-balance point so each scenario starts calmly.
        const Eigen::VectorXd initialInput = CalculateTrimLikeInput(parameters, initialOutput, mass, altitude);

        Eigen::VectorXd state(10);
        state <<
            initialOutput(0),
            initialOutput(1),
            initialOutput(2),
            mass,
            0.0,
            0.0,
            -altitude,
            initialInput(0),
            initialInput(1),
            initialInput(2);
        return state;
    }

    // Reduced model used for LQR design: [airspeed, heading, flight-path angle].
    Eigen::VectorXd ReducedDerivative(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Eigen::VectorXd& state,
        const Eigen::VectorXd& input,
        double mass,
        double altitude)
    {
        const double airspeed = state(0);
        const double flightPathAngle = state(2);
        const double thrust = input(0);
        const double turnAngle = input(1);
        const double diveAngle = input(2);
        const double gravity = CalculateGravity(parameters, altitude);
        const double drag = CalculateDrag(parameters, airspeed, altitude);

        Eigen::VectorXd derivative(3);
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

    // Calculate continuous A and B with centered finite differences.
    void LinearizeReducedModel(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Eigen::VectorXd& trimState,
        const Eigen::VectorXd& trimInput,
        double trimMass,
        double trimAltitude,
        Eigen::MatrixXd& A,
        Eigen::MatrixXd& B)
    {
        A = Eigen::MatrixXd::Zero(3, 3);
        B = Eigen::MatrixXd::Zero(3, 3);

        for (int ii = 0; ii < 3; ii++){
            const double perturbation =
                1e-5 * std::max(std::abs(trimState(ii)), 1.0);
            Eigen::VectorXd statePlus = trimState;
            Eigen::VectorXd stateMinus = trimState;
            statePlus(ii) += perturbation;
            stateMinus(ii) -= perturbation;

            A.col(ii) = (
                ReducedDerivative(parameters, statePlus, trimInput, trimMass, trimAltitude)
                - ReducedDerivative(parameters, stateMinus, trimInput, trimMass, trimAltitude)
                ) / (2.0 * perturbation);
        }

        for (int ii = 0; ii < 3; ii++){
            const double perturbation =
                1e-5 * std::max(std::abs(trimInput(ii)), 1.0);
            Eigen::VectorXd inputPlus = trimInput;
            Eigen::VectorXd inputMinus = trimInput;
            inputPlus(ii) += perturbation;
            inputMinus(ii) -= perturbation;

            B.col(ii) = (
                ReducedDerivative(parameters, trimState, inputPlus, trimMass, trimAltitude)
                - ReducedDerivative(parameters, trimState, inputMinus, trimMass, trimAltitude)
                ) / (2.0 * perturbation);
        }
    }

    void WriteLog(
        const nlohmann::json& log,
        const std::filesystem::path& logPath)
    {
        std::ofstream outputFile(logPath);
        outputFile << "{\n";

        const std::vector<std::string> headerFields{
            "scenarioName",
            "scenarioDescription",
            "controller",
            "dt",
            "finalTime",
            "transitionTime",
            "initialState",
            "linearizationState",
            "linearizationInput",
            "referenceState",
            "referenceInput",
            "initialReferenceState",
            "finalReferenceState",
            "windNed",
            "scheduleSize",
            "scheduleDescription"
        };

        for (const std::string& field : headerFields){
            if (log.contains(field)){
                outputFile << "  \"" << field << "\": "
                           << log[field].dump() << ",\n";
            }
        }

        outputFile << "  \"samples\": [\n";

        for (std::size_t ii = 0; ii < log["samples"].size(); ii++){
            outputFile << "    " << log["samples"][ii].dump();
            if (ii + 1 < log["samples"].size()){
                outputFile << ",";
            }
            outputFile << "\n";
        }

        outputFile << "  ]\n";
        outputFile << "}\n";
    }

    Draft::Autonomy::Control::LQR BuildLqrController(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Eigen::VectorXd& linearizationState,
        const Eigen::VectorXd& linearizationInput,
        double mass,
        double altitude,
        Eigen::MatrixXd& A,
        Eigen::MatrixXd& B)
    {
        LinearizeReducedModel(parameters, linearizationState, linearizationInput, mass, altitude, A, B);

        // These weights encode the tracking errors and command changes we are
        // willing to tolerate around the local linear model.
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(3, 3);
        Q(0, 0) = 1.0 / std::pow(80.0, 2);
        Q(1, 1) = 1.0 / std::pow(Draft::Util::Math::Degrees2Radians(15.0), 2);
        Q(2, 2) = 1.0 / std::pow(Draft::Util::Math::Degrees2Radians(6.0), 2);

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);
        R(0, 0) = 1.0 / std::pow(200000.0, 2);
        R(1, 1) = 1.0 / std::pow(Draft::Util::Math::Degrees2Radians(35.0), 2);
        R(2, 2) = 1.0 / std::pow(Draft::Util::Math::Degrees2Radians(45.0), 2);

        return Draft::Autonomy::Control::LQR(A, B, Q, R);
    }

    Eigen::VectorXd RunLqrTrial(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Draft::Autonomy::Control::LQR& controller,
        const Scenario& scenario,
        const Eigen::VectorXd& initialState,
        const Eigen::VectorXd& linearizationState,
        const Eigen::VectorXd& linearizationInput,
        const Eigen::VectorXd& initialReference,
        const Eigen::VectorXd& finalReference,
        const Eigen::VectorXd& referenceInput,
        const Eigen::VectorXd& windNed,
        double dt,
        const std::filesystem::path& logPath)
    {
        Draft::Dynamics::QE3DofMissile dynamics(parameters);
        Eigen::VectorXd state = initialState;
        const int numSteps =
            static_cast<int>(std::round(scenario.finalTime / dt));

        nlohmann::json log;
        log["scenarioName"] = scenario.name;
        log["scenarioDescription"] = scenario.description;
        log["controller"] = "LQR";
        log["dt"] = dt;
        log["finalTime"] = scenario.finalTime;
        log["transitionTime"] = scenario.transitionTime;
        log["initialState"] = ToStdVector(initialState);
        log["linearizationState"] = ToStdVector(linearizationState);
        log["linearizationInput"] = ToStdVector(linearizationInput);
        log["referenceState"] = ToStdVector(finalReference);
        log["referenceInput"] = ToStdVector(referenceInput);
        log["initialReferenceState"] = ToStdVector(initialReference);
        log["finalReferenceState"] = ToStdVector(finalReference);
        log["windNed"] = ToStdVector(windNed);
        log["samples"] = nlohmann::json::array();

        for (int step = 0; step <= numSteps; step++){
            const double time = step * dt;
            const ReferenceCommand reference = CalculateLinearReference(time, scenario.transitionTime, initialReference, finalReference);
            Eigen::VectorXd reducedState = state.head(3);

            // Keep heading error on the shortest angular path.
            reducedState(1) = reference.output(1)
                + Draft::Util::Math::WrapAngleToPi(
                    reducedState(1) - reference.output(1));

            Eigen::VectorXd command = controller.ComputeControl(reducedState, reference.output, referenceInput);
            command(0) = Draft::Util::Math::Saturate(command(0), 0.0, parameters.maxThrust);
            command(1) = Draft::Util::Math::Saturate(command(1), -parameters.maxTurnAngle, parameters.maxTurnAngle);
            command(2) = Draft::Util::Math::Saturate(command(2), -parameters.maxDiveAngle, parameters.maxDiveAngle);

            log["samples"].push_back({
                {"time", time},
                {"state", ToStdVector(state)},
                {"referenceOutput", ToStdVector(reference.output)},
                {"commandedInput", ToStdVector(command)},
                {"achievedInput", ToStdVector(state.segment(7, 3))}
            });

            if (step < numSteps){
                dynamics.Step(time, dt, state, command, windNed);
            }
        }

        WriteLog(log, logPath);
        return state;
    }

    Eigen::VectorXd RunNdiTrial(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Draft::Autonomy::Control::QE3DofMissileNDI& controller,
        const Scenario& scenario,
        const Eigen::VectorXd& initialState,
        const Eigen::Vector3d& initialReference,
        const Eigen::Vector3d& finalReference,
        const Eigen::VectorXd& windNed,
        double dt,
        const std::filesystem::path& logPath)
    {
        Draft::Dynamics::QE3DofMissile dynamics(parameters);
        Eigen::VectorXd state = initialState;
        const int numSteps =
            static_cast<int>(std::round(scenario.finalTime / dt));

        nlohmann::json log;
        log["scenarioName"] = scenario.name;
        log["scenarioDescription"] = scenario.description;
        log["controller"] = "NDI";
        log["dt"] = dt;
        log["finalTime"] = scenario.finalTime;
        log["transitionTime"] = scenario.transitionTime;
        log["initialState"] = ToStdVector(initialState);
        log["initialReferenceState"] = ToStdVector(initialReference);
        log["finalReferenceState"] = ToStdVector(finalReference);
        log["windNed"] = ToStdVector(windNed);
        log["samples"] = nlohmann::json::array();

        for (int step = 0; step <= numSteps; step++){
            const double time = step * dt;
            const ReferenceCommand reference = CalculateLinearReference(time, scenario.transitionTime, initialReference, finalReference);

            Eigen::VectorXd command = controller.ComputeCommand(state, reference.output, reference.outputRate, reference.outputAcceleration);

            log["samples"].push_back({
                {"time", time},
                {"state", ToStdVector(state)},
                {"referenceOutput", ToStdVector(reference.output)},
                {"referenceOutputRate", ToStdVector(reference.outputRate)},
                {"referenceOutputAcceleration", ToStdVector(reference.outputAcceleration)},
                {"commandedInput", ToStdVector(command)},
                {"achievedInput", ToStdVector(state.segment(7, 3))}
            });

            if (step < numSteps){
                dynamics.Step(time, dt, state, command, windNed);
            }
        }

        WriteLog(log, logPath);
        return state;
    }

    Eigen::VectorXd RunGainScheduledLqrTrial(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Draft::Autonomy::Control::QE3DofMissileGainScheduledLQR& controller,
        const Scenario& scenario,
        const Eigen::VectorXd& initialState,
        const Eigen::Vector3d& initialReference,
        const Eigen::Vector3d& finalReference,
        const Eigen::VectorXd& windNed,
        double dt,
        const std::filesystem::path& logPath)
    {
        Draft::Dynamics::QE3DofMissile dynamics(parameters);
        Eigen::VectorXd state = initialState;
        const int numSteps =
            static_cast<int>(std::round(scenario.finalTime / dt));

        nlohmann::json log;
        log["scenarioName"] = scenario.name;
        log["scenarioDescription"] = scenario.description;
        log["controller"] = "Gain-Scheduled LQR";
        log["dt"] = dt;
        log["finalTime"] = scenario.finalTime;
        log["transitionTime"] = scenario.transitionTime;
        log["initialState"] = ToStdVector(initialState);
        log["initialReferenceState"] = ToStdVector(initialReference);
        log["finalReferenceState"] = ToStdVector(finalReference);
        log["windNed"] = ToStdVector(windNed);
        log["scheduleSize"] = controller.GetScheduleSize();
        log["scheduleDescription"] =
            "Nearest-neighbor schedule using commanded Mach, current altitude, current mass, and commanded flight-path angle.";
        log["samples"] = nlohmann::json::array();

        for (int step = 0; step <= numSteps; step++){
            const double time = step * dt;
            const ReferenceCommand reference = CalculateLinearReference(time, scenario.transitionTime, initialReference, finalReference);

            Eigen::VectorXd command = controller.ComputeCommand(state, reference.output);

            log["samples"].push_back({
                {"time", time},
                {"state", ToStdVector(state)},
                {"referenceOutput", ToStdVector(reference.output)},
                {"commandedInput", ToStdVector(command)},
                {"achievedInput", ToStdVector(state.segment(7, 3))}
            });

            if (step < numSteps){
                dynamics.Step(time, dt, state, command, windNed);
            }
        }

        WriteLog(log, logPath);
        return state;
    }

    void PrintFinalError(
        const std::string& label,
        const Eigen::VectorXd& finalState,
        const Eigen::Vector3d& finalReference)
    {
        std::cout << label << " final airspeed error: "
                  << finalState(0) - finalReference(0) << " m/s\n";
        std::cout << label << " final heading error: "
                  << Draft::Util::Math::Radians2Degrees(
                        Draft::Util::Math::WrapAngleToPi(finalState(1) - finalReference(1)))
                  << " deg\n";
        std::cout << label << " final flight-path angle error: "
                  << Draft::Util::Math::Radians2Degrees(
                        finalState(2) - finalReference(2))
                  << " deg\n";
    }
}

int main()
{
    Draft::Dynamics::QE3DofMissileParameters parameters{
        .earthRadius = 6371000.0,
        .standardGravity = 9.80665,
        .seaLevelDensity = 1.225,
        .referenceHeight = 7200.0,
        .specificHeatRatio = 1.4,
        .idealGasConstant = 287.05,
        .referenceArea = 6.0,
        .subsonicDragConstant = 0.15,
        .supersonicDragConstant = 1.3,
        .specificImpulse = 300.0,
        .dryMass = 8000.0,
        .wetMass = 30000.0,
        .thrustFrequency = 1.5,
        .turnFrequency = 1.0,
        .diveFrequency = 1.0,
        .maxThrust = 500000.0,
        .maxTurnAngle = Draft::Util::Math::Degrees2Radians(40.0),
        .maxDiveAngle = Draft::Util::Math::Degrees2Radians(75.0)
    };

    const std::filesystem::path logDirectory =
        std::filesystem::path(__FILE__).parent_path() / "logs";
    std::filesystem::create_directories(logDirectory);

    constexpr double TimeStep = 0.005;
    constexpr double WindSpeed = 15.0;
    constexpr double WindDirection = 0.7853981633974483;

    Eigen::VectorXd windNed(3);
    windNed <<
        WindSpeed * std::cos(WindDirection),
        WindSpeed * std::sin(WindDirection),
        0.0;

    const std::vector<Scenario> scenarios{
        {
            .name = "scenario1_level_heading",
            .description =
                "Mach 5 level flight at 15 km with a 0 to 20 deg heading change.",
            .altitude = 15000.0,
            .mass = 20000.0,
            .initialMach = 5.0,
            .finalMach = 5.0,
            .initialHeading = Draft::Util::Math::Degrees2Radians(0.0),
            .finalHeading = Draft::Util::Math::Degrees2Radians(20.0),
            .initialFlightPathAngle = Draft::Util::Math::Degrees2Radians(0.0),
            .finalFlightPathAngle = Draft::Util::Math::Degrees2Radians(0.0),
            .finalTime = 100.0,
            .transitionTime = 40.0,
            .runLqr = true
        },
        {
            .name = "scenario2_boost_to_space",
            .description =
                "Full-mass boost from Mach 0.7 to Mach 4 at 9 km while turning from 73 to 90 deg and climbing to 20 deg.",
            .altitude = 9000.0,
            .mass = parameters.wetMass,
            .initialMach = 0.7,
            .finalMach = 4.0,
            .initialHeading = Draft::Util::Math::Degrees2Radians(73.0),
            .finalHeading = Draft::Util::Math::Degrees2Radians(90.0),
            .initialFlightPathAngle = Draft::Util::Math::Degrees2Radians(0.0),
            .finalFlightPathAngle = Draft::Util::Math::Degrees2Radians(20.0),
            .finalTime = 100.0,
            .transitionTime = 100.0,
            .runLqr = false
        },
        {
            .name = "scenario3_powered_dive",
            .description =
                "Powered dive from 40 km, Mach 5 to Mach 8, heading 80 to 60 deg, flight-path angle 0 to -10 deg.",
            .altitude = 40000.0,
            .mass = 20000.0,
            .initialMach = 5.0,
            .finalMach = 8.0,
            .initialHeading = Draft::Util::Math::Degrees2Radians(80.0),
            .finalHeading = Draft::Util::Math::Degrees2Radians(60.0),
            .initialFlightPathAngle = Draft::Util::Math::Degrees2Radians(0.0),
            .finalFlightPathAngle = Draft::Util::Math::Degrees2Radians(-10.0),
            .finalTime = 100.0,
            .transitionTime = 100.0,
            .runLqr = false
        }
    };

    Draft::Autonomy::Control::QE3DofMissileNDI::Gains ndiGains{
        .airspeedNaturalFrequency = 0.35,
        .headingNaturalFrequency = 0.25,
        .flightPathNaturalFrequency = 0.25,
        .airspeedDampingRatio = 1.0,
        .headingDampingRatio = 1.0,
        .flightPathDampingRatio = 1.0
    };
    Draft::Autonomy::Control::QE3DofMissileNDI ndiController(parameters, ndiGains);

    // The gain schedule is intentionally small and uses nearest-neighbor
    // lookup so it stays easy to explain in the report.
    Draft::Autonomy::Control::QE3DofMissileGainScheduledLQR::Grid
        scheduledLqrGrid{
            .mach = {0.7, 1.5, 3.0, 5.0, 6.5, 8.0},
            .altitude = {9000.0, 15000.0, 25000.0, 40000.0},
            .mass = {parameters.dryMass, 15000.0, 20000.0, parameters.wetMass},
            .flightPathAngle = {
                Draft::Util::Math::Degrees2Radians(-10.0),
                Draft::Util::Math::Degrees2Radians(0.0),
                Draft::Util::Math::Degrees2Radians(10.0),
                Draft::Util::Math::Degrees2Radians(20.0)
            }
        };

    Draft::Autonomy::Control::QE3DofMissileGainScheduledLQR::Weights
        scheduledLqrWeights{
            .airspeedError = 80.0,
            .headingError = Draft::Util::Math::Degrees2Radians(15.0),
            .flightPathError = Draft::Util::Math::Degrees2Radians(6.0),
            .thrustCommand = 200000.0,
            .turnCommand = Draft::Util::Math::Degrees2Radians(35.0),
            .diveCommand = Draft::Util::Math::Degrees2Radians(45.0)
        };

    // Build this once. Runtime lookup selects the closest grid point; it does
    // not interpolate or retrim during the simulation.
    Draft::Autonomy::Control::QE3DofMissileGainScheduledLQR
        scheduledLqrController(parameters, scheduledLqrGrid, scheduledLqrWeights);

    std::cout << "Gain-scheduled LQR points: "
              << scheduledLqrController.GetScheduleSize() << "\n";
    std::cout << "Wind NED: " << windNed.transpose() << " m/s\n";

    // Each scenario is run from the same initial state for all controllers.
    // Fixed LQR is only useful near its design point, while scheduled LQR is
    // attempted on every case to show where scheduling helps and where it
    // still runs into model or actuator limits.
    for (const Scenario& scenario : scenarios){
        const Eigen::Vector3d initialReference =
            MakeOutput(parameters, scenario, false);
        const Eigen::Vector3d finalReference =
            MakeOutput(parameters, scenario, true);
        const Eigen::VectorXd initialState = MakeInitialState(parameters, initialReference, scenario.mass, scenario.altitude);

        std::cout << "\nRunning " << scenario.name << "\n";
        std::cout << scenario.description << "\n";
        std::cout << "Initial actuator thrust: "
                  << initialState(7) << " N\n";
        std::cout << "Initial actuator dive angle: "
                  << Draft::Util::Math::Radians2Degrees(initialState(9))
                  << " deg\n";

        if (scenario.runLqr){
            const Eigen::VectorXd lqrReferenceInput =
                CalculateTrimLikeInput(parameters, finalReference, scenario.mass, scenario.altitude);
            Eigen::MatrixXd A;
            Eigen::MatrixXd B;
            Draft::Autonomy::Control::LQR lqrController =
                BuildLqrController(parameters, finalReference, lqrReferenceInput, scenario.mass, scenario.altitude, A, B);

            Eigen::MatrixXd controllability(3, 9);
            controllability << B, A * B, A * A * B;
            const Eigen::MatrixXd closedLoop =
                A - B * lqrController.GetGain();
            const Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(closedLoop, false);

            std::cout << "LQR controllability rank: "
                      << Eigen::FullPivLU<Eigen::MatrixXd>(controllability).rank()
                      << "\n";
            std::cout << "LQR reduced closed-loop poles:\n"
                      << eigenSolver.eigenvalues() << "\n";

            const Eigen::VectorXd finalLqrState =
                RunLqrTrial(
                    parameters,
                    lqrController,
                    scenario,
                    initialState,
                    finalReference,
                    lqrReferenceInput,
                    initialReference,
                    finalReference,
                    lqrReferenceInput,
                    windNed,
                    TimeStep,
                    logDirectory / (scenario.name + "_lqr.json"));
            PrintFinalError(scenario.name + " LQR", finalLqrState, finalReference);
        }

        const Eigen::VectorXd finalScheduledLqrState =
            RunGainScheduledLqrTrial(
                parameters,
                scheduledLqrController,
                scenario,
                initialState,
                initialReference,
                finalReference,
                windNed,
                TimeStep,
                logDirectory
                    / (scenario.name + "_gain_scheduled_lqr.json"));
        PrintFinalError(scenario.name + " Gain-Scheduled LQR", finalScheduledLqrState, finalReference);

        const Eigen::VectorXd finalNdiState =
            RunNdiTrial(
                parameters,
                ndiController,
                scenario,
                initialState,
                initialReference,
                finalReference,
                windNed,
                TimeStep,
                logDirectory / (scenario.name + "_ndi.json"));
        PrintFinalError(scenario.name + " NDI", finalNdiState, finalReference);
    }

    return 0;
}
