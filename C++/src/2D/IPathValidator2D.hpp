/*
 * Author: Andrew Campbell
 * Date: 08-24-2024
 */

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"

// STL
#include <deque>
#include <vector>

namespace Algorithms::TwoD
{
    struct VehicleFeature
    {
        PointXY positionFromCG;
        float width;
        float length;
    };

    class IPathValidator2D
    {
        public:

            virtual bool validatePath(std::deque<PointXY>);
            virtual void setVehicle(std::vector<VehicleFeature>);
    };
}