/*
 * Author: Andrew Campbell
 * Date: 10-14-2024
 */


// TODO: Look into convex hull algorithms
// TODO: remove the grid as a member

#include "RayTracingValidator.hpp"

// Debug
#ifdef VALIDATOR_VISUALIZATION_DEBUG
    #include "Util/Visualization/2DVisualizer.hpp"                
#endif
namespace Algorithms::TwoD
{
    RayTracingValidator::RayTracingValidator(const Configuration::Config& Config,
                                             std::shared_ptr<GridManager2D> Grid,
                                             std::vector<VehicleFeature>& vehicle):
                                             _grid(std::move(Grid))
    {
        setVehicle(vehicle);
    }

    bool RayTracingValidator::validatePath(const std::deque<PointXY>& path, float time)
    {
        if(path.size() < 2)
            return false;
        else if(path.size() == 2)
            return validatePathSegment(path[0], path[1], time);
        else
        {
            for(std::size_t i = 1; i < path.size(); i++)
            {
                // Validate the segment between points
                if(!validatePathSegment(path[i-1], path[i], time))
                    return false;
            }

        return true;
        }
    } // validatePath

    bool RayTracingValidator::validatePathSegment(const PointXY& head, const PointXY& tail, float time)
    {
        // Calculate deltas and angle
        const auto xDelta = tail.x - head.x;
        const auto yDelta = tail.y - head.y;

        // Calculate center point of path
        const PointXY midPoint = head + PointXY(xDelta / 2.0f, yDelta / 2.0f);

        // Calculate the length of the path
        const auto pathLength = PointXY(xDelta, yDelta).norm();

        // If xDelta is 0, the arc-tangent math will error, so set theta manually
        float pathTheta;
        if(xDelta == 0.0f)
        {
            if(yDelta > 0.0f)
                pathTheta = M_PI_2;
            else
                pathTheta = -M_PI_2;
        }
        else
            pathTheta = std::atan(yDelta / xDelta);

        for(auto& feature : _vehicle)
        {
            // Calculate the outline of the path

            // temporary error handling
            if(feature.featurePolygon.size() != 4)
            {
                throw(std::runtime_error("validator not given rectangle"));
            } 

            const auto& outline = Math::Geometry::generateRectangularOutline(midPoint, pathLength + std::abs(feature.featurePolygon[0].x * 2), std::abs(feature.featurePolygon[0].y * 2),pathTheta);
            auto gridCells = _grid->getCells(time);
            // TODO: grid obstacles go here for optimization 

            // Calculate the bounds of the path to remove cells that aren't even in the x/y bounds of the path
            const float xMax = std::max(std::max(outline[0].x, outline[1].x), std::max(outline[2].x, outline[3].x));
            const float xMin = std::min(std::min(outline[0].x, outline[1].x), std::min(outline[2].x, outline[3].x));
            const float yMax = std::max(std::max(outline[0].y, outline[1].y), std::max(outline[2].y, outline[3].y));
            const float yMin = std::min(std::min(outline[0].y, outline[1].y), std::min(outline[2].y, outline[3].y));
            
            #ifdef VALIDATOR_VISUALIZATION_DEBUG
                auto debugVisaulizer = Visualization::Visualizer2D(_grid);
                debugVisaulizer.plotGrid(time);
                debugVisaulizer.plotPolygon(outline);
                debugVisaulizer.show("Validator Bounding Box");
            #endif

            // Can't do much if there is no grid
            if(gridCells.empty()){
                throw std::runtime_error("validator asked to validate on an empty grid");
                return false;
            }
            for(const auto& cell : gridCells)
            {
                // Safety check for safety
                const auto& cellOutline = cell.getCorners();
                if(cellOutline.size() != 4)
                    return false; // this must be done until convex hull is implemented

                if(cellOutline[0].x > xMax || cellOutline[3].x < xMin || cellOutline[0].y > yMax || cellOutline[3].y < yMin)
                    continue;
                // Check that the cell state is correct and the cell outline is in the bounds of the path before doing computation
                // Cell outline is in the same order for every cell, with index 0 at the top left and 3 at the bottom right
                
                const auto cellState = cell.getState();
                if(cellState == State::OBSTACLE)
                {
                    // Check the 4 corners
                    // TODO: check if running the polygon intersect is faster than running this four times
                    if(Math::Geometry::isPointInsidePolygon(cellOutline[0], outline) ||
                       Math::Geometry::isPointInsidePolygon(cellOutline[1], outline) ||
                       Math::Geometry::isPointInsidePolygon(cellOutline[2], outline) ||
                       Math::Geometry::isPointInsidePolygon(cellOutline[3], outline)   )
                           return false;
                }
            }

        }

        return true;
    } // validatePathSegment

    bool RayTracingValidator::validatePose(const StateXYT& state, float time)
    {

        for(auto& feature : _vehicle)
        {
            // Calculate the outline of the path

            // temporary error handling
            if(feature.featurePolygon.size() != 4)
            {
                throw(std::runtime_error("validator not given rectangle"));
            } 

            const auto& outline = Math::Geometry::generateRectangularOutline(PointXY(state.x, state.y), std::abs(feature.featurePolygon[0].x * 2), std::abs(feature.featurePolygon[0].y * 2), state.z);
            auto gridCells = _grid->getCells(time);
            // TODO: grid obstacles go here for optimization 

            // Calculate the bounds of the path to remove cells that aren't even in the x/y bounds of the path
            const float xMax = std::max(std::max(outline[0].x, outline[1].x), std::max(outline[2].x, outline[3].x));
            const float xMin = std::min(std::min(outline[0].x, outline[1].x), std::min(outline[2].x, outline[3].x));
            const float yMax = std::max(std::max(outline[0].y, outline[1].y), std::max(outline[2].y, outline[3].y));
            const float yMin = std::min(std::min(outline[0].y, outline[1].y), std::min(outline[2].y, outline[3].y));
            
            #ifdef VALIDATOR_VISUALIZATION_DEBUG
                auto debugVisaulizer = Visualization::Visualizer2D(_grid);
                debugVisaulizer.plotPolygon(outline);
                debugVisaulizer.show("Validator Bounding Box");
            #endif

            // Can't do much if there is no grid
            if(gridCells.empty()){
                throw std::runtime_error("validator asked to validate on an empty grid");
                return false;
            }
            for(const auto& cell : gridCells)
            {
                // Safety check for safety
                const auto& cellOutline = cell.getCorners();
                if(cellOutline.size() != 4)
                    return false; // this must be done until convex hull is implemented

                if(cellOutline[0].x > xMax || cellOutline[3].x < xMin || cellOutline[0].y > yMax || cellOutline[3].y < yMin)
                    continue;
                // Check that the cell state is correct and the cell outline is in the bounds of the path before doing computation
                // Cell outline is in the same order for every cell, with index 0 at the top left and 3 at the bottom right
                
                const auto cellState = cell.getState();
                if(cellState == State::OBSTACLE)
                {
                    // Check the 4 corners
                    // TODO: check if running the polygon intersect is faster than running this four times
                    // TODO: this has an edgecase of the vehicle corner just barely dips into an obstacle but only on an edge
                    // a robust solution is checking poligonal intersections but It may be more computationally expensive 
                    if(Math::Geometry::isPointInsidePolygon(cellOutline[0], outline) ||
                       Math::Geometry::isPointInsidePolygon(cellOutline[1], outline) ||
                       Math::Geometry::isPointInsidePolygon(cellOutline[2], outline) ||
                       Math::Geometry::isPointInsidePolygon(cellOutline[3], outline)   )
                           return false;
                }
            }
        }
        
        return true;
    } // validatePose


    void RayTracingValidator::setVehicle(std::vector<VehicleFeature>& vehicle)
    {
        _vehicle = vehicle;

        float minimumDistance = INFINITY;
        // calculate minimum safe distance
        int numSamples = 100;
        for(int i = 0; i < numSamples; i++)
        {
            float angle = M_PI * 2 / numSamples;
            float xMin, xMax, yMin, yMax;
            _grid->getGridDomain(xMin, xMax, yMax, yMax); // dunno
            float rayLength = std::sqrt( std::pow((xMax - xMin), 2) + std::pow((yMax - yMin), 2));
            auto ray = std::make_pair<PointXY, PointXY>(PointXY(0.0,0.0), PointXY(rayLength * std::cos(angle), rayLength * std::sin(angle)));

            float featureMaxDistance = -1;
            for (auto& feature : _vehicle)
            {
                for(int i = 0; i < feature.featurePolygon.size(); i++)
                {
                    float distance = -2;
                    if(i == feature.featurePolygon.size() - 1)
                    {
                        auto line = std::make_pair<PointXY, PointXY>(PointXY(feature.featurePolygon[i]),PointXY(feature.featurePolygon[0]));
                        if(Math::Geometry::doTwoFiniteLinesIntersect(ray,line))
                        {
                            PointXY intersect = Math::Geometry::linesIntersection(ray,line);
                            float distance = std::sqrt(std::pow(intersect.x,2) + std::pow(intersect.y,2));
                        }
                    }
                    else
                    {
                        auto line = std::make_pair<PointXY, PointXY>(PointXY(feature.featurePolygon[i]), PointXY(feature.featurePolygon[i + 1]));
                        if(Math::Geometry::doTwoFiniteLinesIntersect(ray,line))
                        {
                            PointXY intersect = Math::Geometry::linesIntersection(ray,line);
                            float distance = std::sqrt(std::pow(intersect.x,2) + std::pow(intersect.y,2));
                        }
                    }

                    if(distance > featureMaxDistance)
                    {
                        featureMaxDistance = distance;
                    }

                }
            }

            if(featureMaxDistance >= 0 && featureMaxDistance < minimumDistance)
            {
                minimumDistance = featureMaxDistance;
            }
        }

        _minimumSafeDistance = minimumDistance;
        
    } // setVehicle

    const float RayTracingValidator::getMinimumSafeDistance(){ return _minimumSafeDistance; } 
    // TODO: this could be used as a preprocessing step to populate the grid in an artificial manner for the goal of initial point sampling in RRT* and RRT sharp
}