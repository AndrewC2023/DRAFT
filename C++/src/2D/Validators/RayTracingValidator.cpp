/*
 * Author: Andrew Campbell
 * Date: 10-14-2024
 */

#include "RayTracingValidator.hpp"

namespace Algorithms::TwoD
{
    RayTracingValidator::RayTracingValidator(Configuration::Config& Config,
                                             std::shared_ptr<GridManager> Grid):
                                             _grid(std::move(Grid))
    {

    }

    RayTracingValidator::validatePath(std::deque<PointXY> path)
    {

    }

    RayTracingValidator::validatePathSegment(PointXY startPoint, PointXY endPoint)
    {

    }

    RayTracingValidator::setVehicle(std::vector<VehicleFeature> vehicle)
    {
        _vehicle = vehicle;
    }

    RayTracingValidator::getMinimumSafeDistance(){ return _minimumSafeDistance; }
}