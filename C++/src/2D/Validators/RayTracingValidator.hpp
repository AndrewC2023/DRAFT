/*
 * Author: Andrew Campbell
 * Date: 10-09-2024
 */

#ifndef RAYTRACINGVALIDATOR_H
#define RAYTRACINGVALIDATOR_H

// Config
#include "Util/Config/Config.hpp"

// Base class
#include "../IPathValidator2D.hpp"

// STL
#include <cmath>
#include <memory>
#include <deque>

#include "Util/Visualization/2DVisualizer.hpp"

// TODO: there is so much to work on with this algorithm to make it more robust

namespace Algorithms::TwoD
{
    class RayTracingValidator : public IPathValidator2D

    {
        public:
            RayTracingValidator(const Configuration::Config&,
                                std::shared_ptr<GridManager2D>,
                                std::vector<VehicleFeature>&);
                                
            ~RayTracingValidator();

            bool validatePath(const std::deque<PointXY>&, float time) override;
            bool validatePathSegment(const PointXY&, const PointXY&, float time) override;
            void setVehicle(std::vector<VehicleFeature>&) override;
            const float getMinimumSafeDistance() override;
            bool validatePose(const StateXYT&, float time ) override;

        private:

            // variables
            std::deque<PointXY> _path;
            std::vector<VehicleFeature> _vehicle;

            // debug
            bool _debug;
            Visualization::Visualizer2D* debugVisaulizer;
            bool _gridGraphed;

            // grid
            std::shared_ptr<GridManager2D> _grid;

            float _minimumSafeDistance; 

    };

}


#endif // RAYTRACINGVALIDATOR_H
