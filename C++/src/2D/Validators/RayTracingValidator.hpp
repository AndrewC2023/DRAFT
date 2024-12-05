/*
 * Author: Andrew Campbell
 * Date: 10-09-2024
 */

#ifndef RAYTRACINGVALIDATOR_H
#define RAYTRACINGVALIDATOR_H

// Config
#include "Util/Config/Config.hpp"

// Base class
#include "IPathValidator2D.hpp"

// STL
#include <cmath>
#include <memory>
#include <deque>

namespace Algorithms::TwoD
{
    class RayTracingValidator : IPathValidator2D

    {
        public:
            RayTracingValidator(Configuration::Config&,
                                std::shared_ptr<GridManager2D>);
                                
            ~RayTracingValidator() = default;

            bool validatePath(std::deque<PointXY>) override;
            bool validatePathSegment(PointXY, PointXY) override;
            void setVehicle(std::vector<VehicleFeature>) override;
            float getMinimumSafeDistance() override;

        private:

            // variables
            std::deque<PointXY> _path;
            std::vector<VehicleFeature> _vehicle;

            // grid
            std::shared_ptr<GridManager2D> _grid;

            float _minimumSafeDistance; 

    };

}


#endif // RAYTRACINGVALIDATOR_H
