/*
 * Author: Andrew Campbell
 * Date: 06-14-2026
 */

#ifndef DYNAMICSBASE_H
#define DYNAMICSBASE_H

// Eigen
#include <Eigen/Dense>

namespace Draft::Dynamics
{
    class DynamicsBase
    {
        public:
            virtual ~DynamicsBase() = default;

            virtual void Step(double time, double dt, Eigen::VectorXd& state, const Eigen::VectorXd& systemInput, const Eigen::VectorXd& additionalInputs)
                const = 0;
            virtual void Step(double time, double dt, Eigen::VectorXd& state, const Eigen::VectorXd& systemInput) const
            {
                static const Eigen::VectorXd noAdditionalInputs;
                Step(time, dt, state, systemInput, noAdditionalInputs);
            };
            // additional parameters could be environmental inputs such as wind, some sort of noise, unsure rn

        protected:
            virtual void ApplyConstraints([[maybe_unused]] Eigen::VectorXd& state) const {}
    };
}

#endif // DYNAMICSBASE_H