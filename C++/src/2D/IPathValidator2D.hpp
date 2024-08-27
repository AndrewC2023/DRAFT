
// STL
#include <deque>

// Eigen
#include <Eigen/Dense>

#include "Util/CustomTypes/VectorAndPointTypes.hpp"

namespace Algorithms::TwoD
{
    class IPathValidator2D
    {
        public:
            virtual bool validatePath(std::deque<StateXYT>);
    };
}