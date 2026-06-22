/*
 * Author: Andrew Campbell
 * Date: 06-12-2026
 */

#ifndef DYNAMICSBASE_H
#define DYNAMICSBASE_H

// Eigen
#include <Eigen/Dense>

namespace Draft::Dynamics
{
    /**
     * @brief Common interface for stateless dynamics models.
     *
     * The caller owns the state and current time. A dynamics model advances
     * the supplied state in place using the supplied system and optional
     * additional inputs.
     */
    class DynamicsBase
    {
        public:
            /**
             * @brief Destroys a dynamics model through the base interface.
             */
            virtual ~DynamicsBase() = default;

            /**
             * @brief Advances a state by one timestep with additional inputs.
             *
             * @param time Time associated with the supplied state.
             * @param dt Positive integration timestep.
             * @param state State to advance in place.
             * @param systemInput Command or control input vector.
             * @param additionalInputs Optional model-specific inputs such as wind.
             */
            virtual void Step(double time, double dt, Eigen::VectorXd& state, const Eigen::VectorXd& systemInput, const Eigen::VectorXd& additionalInputs)
                const = 0;

            /**
             * @brief Advances a state by one timestep without additional inputs.
             *
             * This convenience overload forwards an empty additional-input
             * vector to the full Step implementation.
             *
             * @param time Time associated with the supplied state.
             * @param dt Positive integration timestep.
             * @param state State to advance in place.
             * @param systemInput Command or control input vector.
             */
            virtual void Step(double time, double dt, Eigen::VectorXd& state, const Eigen::VectorXd& systemInput) const
            {
                static const Eigen::VectorXd noAdditionalInputs;
                Step(time, dt, state, systemInput, noAdditionalInputs);
            };

        protected:
            /**
             * @brief Applies model-specific limits or corrections after a step.
             *
             * The default implementation leaves the state unchanged.
             *
             * @param state Propagated state to constrain in place.
             */
            virtual void ApplyConstraints([[maybe_unused]] Eigen::VectorXd& state) const {}
    };
}

#endif // DYNAMICSBASE_H
