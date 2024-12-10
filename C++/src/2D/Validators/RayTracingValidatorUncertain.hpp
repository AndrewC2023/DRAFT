/*
 * Author: Andrew Campbell
 * Date: 12-01-2024
 */

#ifndef RAYTRACINGVALIDATORUNCERTAIN_H
#define RAYTRACINGVALIDATORUNCERTAIN_H

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
    class RayTracingValidatorUncertain : public IPathValidator2D

    {
        public:
            RayTracingValidatorUncertain(const Configuration::Config&,
                                std::shared_ptr<GridManager2D>,
                                std::vector<VehicleFeature>&);
                                
            ~RayTracingValidatorUncertain();

            bool validatePath(const std::deque<PointXY>&, float time, float& probability) override;
            bool validatePathSegment(const PointXY&, const PointXY&, float time, float& probability) override;
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


#endif // RAYTRACINGVALIDATORUNCERTAIN_H
