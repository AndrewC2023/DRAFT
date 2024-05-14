// STL
#include <deque>
#include <vector>
#include <cmath>

// Eigen
#include <Eigen/Core>

// base class
#include "IPathPlanner3D.hpp"

namespace algorithms::ThreeD::Planners
{
    struct AStarNode{
        Eigen::Vector3f Position;
        int Children[]; // or array potentially
    };

    class AStarGridPlanner
    {
        public:
            std::vector<>
    }
}