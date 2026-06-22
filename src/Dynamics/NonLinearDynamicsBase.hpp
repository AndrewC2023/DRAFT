/*
 * Author: Andrew Campbell
 * Date: 06-13-2026
 */

#ifndef NONLINEARDYNAMICSBASE_H
#define NONLINEARDYNAMICSBASE_H

#include "DynamicsBase.hpp"

// Eigen
#include <Eigen/Dense>

// Internal math
#include "Util/Math/Math.hpp"

// TODO: we should offer several ODE solvers and just use polymorphism

namespace Draft::Dynamics
{
    /**
     * @brief Base class for continuous-time nonlinear dynamics models.
     *
     * Derived classes define the continuous state derivative. This base class
     * advances caller-owned state with fixed-step classical RK4 integration.
     */
    class NonLinearDynamicsBase : public DynamicsBase
    {
        public:
            using DynamicsBase::Step;

            /**
             * @brief Advances a nonlinear state by one timestep.
             *
             * @param time Time associated with the supplied state.
             * @param dt Positive integration timestep.
             * @param state State to advance in place.
             * @param systemInput Command or control input vector.
             * @param additionalInputs Optional model-specific inputs.
             */
            void Step(double time, double dt, Eigen::VectorXd& state, const Eigen::VectorXd& systemInput, const Eigen::VectorXd& additionalInputs) const override
            {
                state = Draft::Util::Math::RungeKutta4(
                    time, 
                    state, 
                    dt, 
                    [this, &systemInput, &additionalInputs](double evaluationTime, const Eigen::VectorXd& evaluationState)
                        { return EvaluateDerivative(evaluationTime, evaluationState, systemInput, additionalInputs);} 
                    );

                ApplyConstraints(state);
            };


            // TODO: Much later we should implenent a way to get the linearized system around some point, eithe trim or non trim, ideally both? do this not today

        protected:
            /**
             * @brief Evaluates the continuous state derivative.
             *
             * @param time Time at which to evaluate the model.
             * @param state State at which to evaluate the model.
             * @param systemInput Command or control input vector.
             * @param environmentalInput Optional environmental/model input vector.
             * @return State derivative with the same dimension as state.
             */
            virtual Eigen::VectorXd EvaluateDerivative(
                double time,
                const Eigen::VectorXd& state,
                const Eigen::VectorXd& systemInput,
                const Eigen::VectorXd& environmentalInput
            ) const = 0;
    };
}

#endif // NONLINEARDYNAMICSBASE_H
