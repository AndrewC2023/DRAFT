/*
 * Author: Andrew Campbell
 * Date: 12-04-2024
 */

#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "Eigen/Dense"

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"

namespace Math::Dynamics
{
    class IDynamics
    {
        virtual ~IDynamics() = default;
    };

    class SISOLinearDynamics
    {
        public:

        private:

            bool controllableCanonicalForm;
    };

    class MIMOLinearDynamics
    {
        public:

        private:
        
    };
}

#endif // IDYNAMICS_H