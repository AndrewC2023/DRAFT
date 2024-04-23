
// STL
#include <deque>
#include <functional>
#include <iostream>

// Eigen
#include <Eigen/Dense>

namespace algorithms::TwoD
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

        virtual std::deque<Eigen::vector3f> PlanPath(Eigen::vector3f start, Eigen::vector3f goal);

    };

}