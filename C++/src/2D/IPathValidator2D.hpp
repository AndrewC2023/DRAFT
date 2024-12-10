/*
 * Author: Andrew Campbell
 * Date: 08-24-2024
 */

#ifndef IPATHVALIDATOR2D_H
#define IPATHVALIDATOR2D_H

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
        PointXY featureCenterfromCG;
        std::vector<PointXY> featurePolygon; /// the polygon given as positions relative to the feature defined reference point
    };

    class IPathValidator2D
    {
        public:
            virtual ~IPathValidator2D() = default;

            virtual bool validatePath(const std::deque<PointXY>&, float time, float& probability) = 0;
            virtual bool validatePathSegment(const PointXY&, const PointXY&, float time, float& probability) = 0;
            virtual const float getMinimumSafeDistance() = 0;
            virtual bool validatePose(const StateXYT&, float time) = 0;
            virtual void setVehicle(std::vector<VehicleFeature>&) = 0;

    };
    
}

#endif // IPATHVALIDATOR2D_H