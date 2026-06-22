/*
 * Author: Andrew Campbell
 * Date: 06-19-2026
 */

// Dynamics
#include "Dynamics/NonLinearModels/QE3DofMissile.hpp"

// Local validation controllers
#include "QE3DofMissileNDI.hpp"

// Eigen
#include <Eigen/Dense>

// STL
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
    // Run the same dynamics case with a selected fixed timestep.
    Eigen::VectorXd RunSimulation(
        const Draft::Dynamics::QE3DofMissileParameters& parameters,
        const Eigen::VectorXd& initialState,
        const Eigen::VectorXd& command,
        double dt,
        double finalTime)
    {
        Draft::Dynamics::QE3DofMissile dynamics(parameters);
        Eigen::VectorXd state = initialState;

        double time = 0.0;
        const int numSteps = static_cast<int>(std::round(finalTime / dt));
        for (int step = 0; step < numSteps; step++){
            dynamics.Step(time, dt, state, command);
            time += dt;
        }

        return state;
    }

    // Scale each state error so large physical units do not hide angle errors.
    double GetScaledMaxError(
        const Eigen::VectorXd& state,
        const Eigen::VectorXd& referenceState)
    {
        const Eigen::ArrayXd scale = referenceState.array().abs().max(1.0);
        return ((state - referenceState).array().abs() / scale).maxCoeff();
    }
}

int main()
{
    try {
        Draft::Dynamics::QE3DofMissileParameters parameters{
            .earthRadius = 6371000.0,
            .standardGravity = 9.80665,
            .seaLevelDensity = 1.225,
            .referenceHeight = 7200.0,
            .specificHeatRatio = 1.4,
            .idealGasConstant = 287.05,
            .referenceArea = 5.0,
            .subsonicDragConstant = 0.15,
            .supersonicDragConstant = 1.25,
            .specificImpulse = 300.0,
            .dryMass = 8000.0,
            .wetMass = 30000.0,
            .thrustFrequency = 1.5,
            .turnFrequency = 1.0,
            .diveFrequency = 1.0,
            .maxThrust = 500000.0,
            .maxTurnAngle = 0.6981317008,
            .maxDiveAngle = 1.3089969390
        };

        // State order:
        // [airspeed, heading, flight-path angle, mass, north, east, down,
        //  achieved thrust, achieved turn angle, achieved dive angle]
        Eigen::VectorXd initialState(10);
        initialState <<
            1000.0,       // airspeed, m/s
            0.10,         // heading, rad
            0.05,         // flight-path angle, rad
            30000.0,      // mass, kg
            0.0,          // north position, m
            0.0,          // east position, m
            -20000.0,     // down position, m (20 km altitude)
            200000.0,     // achieved thrust, N
            0.01,         // achieved turn angle, rad
            -0.01;        // achieved dive angle, rad

        // Command order: [commanded thrust, commanded turn angle,
        // commanded dive angle]
        Eigen::VectorXd command(3);
        command <<
            250000.0,     // commanded thrust, N
            0.02,         // commanded turn angle, rad
            -0.015;       // commanded dive angle, rad

        // Check each first-order actuator against its analytical response.
        constexpr double ActuatorTestTime = 1.0;
        const Eigen::VectorXd actuatorState = RunSimulation(
            parameters, initialState, command, 0.001, ActuatorTestTime);

        const double expectedThrust = command(0)
            + (initialState(7) - command(0))
                * std::exp(-parameters.thrustFrequency * ActuatorTestTime);
        const double expectedTurnAngle = command(1)
            + (initialState(8) - command(1))
                * std::exp(-parameters.turnFrequency * ActuatorTestTime);
        const double expectedDiveAngle = command(2)
            + (initialState(9) - command(2))
                * std::exp(-parameters.diveFrequency * ActuatorTestTime);

        if (std::abs(actuatorState(7) - expectedThrust) > 1e-5
            || std::abs(actuatorState(8) - expectedTurnAngle) > 1e-10
            || std::abs(actuatorState(9) - expectedDiveAngle) > 1e-10){
            std::cerr << "Actuator response validation failed.\n";
            return 1;
        }

        // With constant achieved thrust, mass depletion has a linear solution.
        Eigen::VectorXd massFlowState = initialState;
        Eigen::VectorXd massFlowCommand(3);
        massFlowCommand <<
            initialState(7),
            initialState(8),
            initialState(9);

        constexpr double MassFlowTestTime = 1.0;
        const Eigen::VectorXd finalMassFlowState = RunSimulation(
            parameters,
            massFlowState,
            massFlowCommand,
            0.001,
            MassFlowTestTime);
        const double expectedMass = initialState(3)
            - initialState(7)
                / (parameters.specificImpulse * parameters.standardGravity)
                * MassFlowTestTime;

        if (std::abs(finalMassFlowState(3) - expectedMass) > 1e-8){
            std::cerr << "Mass-flow validation failed.\n";
            return 1;
        }

        Draft::Autonomy::Control::QE3DofMissileNDI::Gains ndiGains{
            .airspeedNaturalFrequency = 0.30,
            .headingNaturalFrequency = 0.20,
            .flightPathNaturalFrequency = 0.20,
            .airspeedDampingRatio = 1.0,
            .headingDampingRatio = 1.0,
            .flightPathDampingRatio = 1.0
        };
        Draft::Autonomy::Control::QE3DofMissileNDI ndiController(
            parameters,
            ndiGains);

        Eigen::VectorXd commandedOutput(3);
        commandedOutput <<
            initialState(0),
            initialState(1),
            initialState(2);
        Eigen::VectorXd commandedOutputRate =
            Eigen::VectorXd::Zero(3);
        Eigen::VectorXd commandedOutputAcceleration =
            Eigen::VectorXd::Zero(3);

        const Eigen::VectorXd ndiCommand =
            ndiController.ComputeCommand(
                initialState,
                commandedOutput,
                commandedOutputRate,
                commandedOutputAcceleration);

        if (ndiCommand.size() != 3 || !ndiCommand.allFinite()){
            std::cerr << "NDI command validation failed.\n";
            return 1;
        }

        constexpr double FinalTime = 10.0;

        // The smallest timestep is used as the reference solution.
        const Eigen::VectorXd state004 = RunSimulation(parameters, initialState, command, 0.04, FinalTime);
        const Eigen::VectorXd state002 = RunSimulation(parameters, initialState, command, 0.02, FinalTime);
        const Eigen::VectorXd state001 = RunSimulation(parameters, initialState, command, 0.01, FinalTime);
        const Eigen::VectorXd referenceState = RunSimulation(parameters, initialState, command, 0.005, FinalTime);

        const double error004 = GetScaledMaxError(state004, referenceState);
        const double error002 = GetScaledMaxError(state002, referenceState);
        const double error001 = GetScaledMaxError(state001, referenceState);

        std::cout << "Actuator response validation passed.\n";
        std::cout << "Mass-flow validation passed.\n";
        std::cout << "NDI command validation passed.\n";
        std::cout << "QE3DofMissile fixed-step RK4 convergence\n";
        std::cout << "dt = 0.040 s, scaled max error = " << error004 << "\n";
        std::cout << "dt = 0.020 s, scaled max error = " << error002 << "\n";
        std::cout << "dt = 0.010 s, scaled max error = " << error001 << "\n";

        if (!(error002 < error004 && error001 < error002)){
            std::cerr << "Convergence validation failed.\n";
            return 1;
        }

        std::cout << "Validation passed.\n";
        return 0;
    }
    catch (const std::exception& exception){
        std::cerr << "Validation failed: " << exception.what() << "\n";
        return 1;
    }
}
