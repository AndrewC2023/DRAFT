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

    bool RayTracingValidator::validatePath(std::deque<PointXY> path)
    {
        if(path.size() < 2)
            return false;
        else if(path.size() == 2)
            return validatePathSegment(path[0], path[1]);
        else
        {
            for(std::size_t i = 1; i < path.size(); i++)
            {
                // Validate the segment between points
                if(!validatePathSegment(path[i-1], path[i]))
                    return false;
            }

        return true;
        }
    }

    bool RayTracingValidator::validatePathSegment(PointXY startPoint, PointXY endPoint)
    {

    }

    void RayTracingValidator::setVehicle(std::vector<VehicleFeature> vehicle)
    {
        _vehicle = vehicle;
    }

    float RayTracingValidator::getMinimumSafeDistance(){ return _minimumSafeDistance; }
}