/*
 * Author: Andrew Campbell
 * Date: 08-24-2024
 */

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"

// Grid
#include "2DGridManager.hpp"

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
            virtual bool validatePathSegment(PointXY, PointXY);
            virtual float getMinimumSafeDistance();
            virtual bool validatePoint(PointXY);
            virtual bool validatePolynomial();
            void setVehicle(std::vector<VehicleFeature>&);

        private:
            std::vector<VehicleFeature> _vehicle;
    };

    void IPathValidator2D::setVehicle(std::vector<VehicleFeature>& vehicle)
    {
        _vehicle = vehicle;
    }
    
}