
// STL
#include <deque>

// Eigen
#include <Eigen/Dense>

namespace Algorithms::ThreeD
{
    class IPathPlanner3D
    {
        virtual std::deque<Eigen::PointXYZ> PlanPath()
    }
}