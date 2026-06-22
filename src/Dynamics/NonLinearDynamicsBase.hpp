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

namespace Draft::Dynamics
{
    class NonLinearDynamicsBase : public DynamicsBase
    {
        public:
            using DynamicsBase::Step;
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
            virtual Eigen::VectorXd EvaluateDerivative(
                double time,
                const Eigen::VectorXd& state,
                const Eigen::VectorXd& systemInput,
                const Eigen::VectorXd& environmentalInput
            ) const = 0;
    };
}

#endif // NONLINEARDYNAMICSBASE_H