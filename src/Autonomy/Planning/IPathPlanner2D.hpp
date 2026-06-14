/*
 * Author: Andrew Campbell
 * Date: 08-24-2024
 */

// STL
#include <deque>
#include <functional>
#include <iostream>

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"

namespace Algorithms::TwoD
{
    struct Penalties{
        const float ObstaclePentalty;
        const float InvalidPenalty;
        const float UnknownPenalty;
    };

    class IPathPlanner2D
    {
        public:
            virtual ~IPathPlanner2D() = default;
            virtual std::deque<PointXY> PlanPath(PointXY start, PointXY goal) = 0;

            virtual float getPathCost() = 0; // Helper to return the cost of the found path
    };

}