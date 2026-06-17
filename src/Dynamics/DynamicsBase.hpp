/*
 * Author: Andrew Campbell
 * Date: 06-14-2026
 */

#ifndef DYNAMICSBASE_H
#define DYNAMICSBASE_H

// Eigen
#include <Eigen/Dense>

// STD
#include <vector>
#include <string>

namespace Draft::Dynamics
{
    class DynamicsBase
    {
        public:
            DynamicsBase() = default;

            explicit DynamicsBase(
                const Eigen::VectorXd& initialState,
                std::vector<std::string> stateNames = {},
                double initialTime = 0.0)
                : state_(initialState),
                  stateNames_(std::move(stateNames)),
                  time_(initialTime)
            {
            }

            virtual ~DynamicsBase() = default;

            // helpers for all children to have
            const Eigen::VectorXd& GetState() const noexcept { return state_; };
            const std::vector<std::string>& GetStateNames() const noexcept { return stateNames_; };
            void SetState(Eigen::VectorXd state) { state_ = state; };
            void SetStateNames(std::vector<std::string> stateNames) { stateNames_ = stateNames; };
            void SetTime(double time) noexcept { time_ = time; };
            const double GetTime() const noexcept { return time_; };
            std::size_t GetStateDim() const noexcept{ return static_cast<std::size_t>(state_.size()); }

            // The all important Step function, children must implement
            virtual Eigen::VectorXd Step(double dt, Eigen::VectorXd SystemInputs) = 0;
            virtual Eigen::VectorXd Step(double dt, Eigen::VectorXd SystemInputs, Eigen::VectorXd EnvironmentalInputs) = 0;

        protected:
            Eigen::VectorXd state_;
            std::vector<std::string> stateNames_;
            double time_ = 0.0;
    };
}

#endif // DYNAMICSBASE_H