
// STL
#include <deque>

// Eigen
#include <Eigen/Dense>

namespace algorithms::TwoD::Planners
{
    class IPathPlanner2D
    {
        public:

        virtual std::deque<Eigen::PointXYZ> PlanPath()
    };

}