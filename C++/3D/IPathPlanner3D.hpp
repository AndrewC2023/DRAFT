
// STL
#include <deque>

// Eigen
#include <Eigen/Dense>

namespace algorithms::ThreeD
{
    class IPathPlanner3D
    {
        virtual std::deque<Eigen::PointXYZ> PlanPath()
    }
}