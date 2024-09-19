
// STL
#include <deque>
#include <functional>
#include <iostream>

// Eigen
#include <Eigen/Dense>

namespace Algorithms::TwoD
{
    typedef struct Penalties{
        ObstaclePentalty;
        InvalidPenalty;
        UnknownPenalty;
    };

    typedef std::function<float>(Eigen::vector3f, Eigen::vector3f, Penalties) CostFunction;

    class IPathPlanner2D
    {
        public:

        virtual std::deque<PointXY> PlanPath(Eigen::vector3f start, Eigen::vector3f goal);

    };

}