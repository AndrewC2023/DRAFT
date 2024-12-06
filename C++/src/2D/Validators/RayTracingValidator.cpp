/*
 * Author: Andrew Campbell
 * Date: 10-14-2024
 */

#include "RayTracingValidator.hpp"

namespace Algorithms::TwoD
{
    RayTracingValidator::RayTracingValidator(Configuration::Config& Config,
                                             std::shared_ptr<GridManager2D> Grid):
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



    bool RayTracingValidator::validatePathSegment(const PointXY& head, const PointXY& tail)
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

        // Calculate the outline of the path
        const auto& outline = navigation::generateRectangularOutline(midPoint.x, midPoint.y, pathTheta, _robotWidth_m + _doubledWidthMargin_m, _robotLength_m + _doubledLengthMargin_m + pathLength);
        auto gridCells = _grid->getCells();

        // Calculate the bounds of the path to remove cells that aren't even in the x/y bounds of the path
        const float xMax = std::max(std::max(outline[0].x, outline[1].x), std::max(outline[2].x, outline[3].x));
        const float xMin = std::min(std::min(outline[0].x, outline[1].x), std::min(outline[2].x, outline[3].x));
        const float yMax = std::max(std::max(outline[0].y, outline[1].y), std::max(outline[2].y, outline[3].y));
        const float yMin = std::min(std::min(outline[0].y, outline[1].y), std::min(outline[2].y, outline[3].y));


        // Can't do much if there is no grid
        if(gridCells.empty())
            return false;

        for(const auto& cell : gridCells)
        {
            // Safety check for safety
            const auto& cellOutline = cell.getOutline();
            if(cellOutline.size() != 4)
                return false;

            if(cellOutline[0].x > xMax || cellOutline[3].x < xMin || cellOutline[0].y > yMax || cellOutline[3].y < yMin)
                continue;
            // Check that the cell state is correct and the cell outline is in the bounds of the path before doing computation
            // Cell outline is in the same order for every cell, with index 0 at the top left and 3 at the bottom right
            
            const auto cellState = cell.getState();
            if((cellState == GridCellState::POSITIVE_OBSTACLE || cellState == GridCellState::NEGATIVE_OBSTACLE || cellState == GridCellState::WALL))
            {
                // Check the 4 corners
                // TODO: check if running the polygon intersect is faster than running this four times
                if(Math::Geometry::isPointInsidePolygon(cellOutline[0], outline) ||
                Math::Geometry::isPointInsidePolygon(cellOutline[1], outline) ||
                Math::Geometry::isPointInsidePolygon(cellOutline[2], outline) ||
                Math::Geometry::isPointInsidePolygon(cellOutline[3], outline))
                    return false;
            }
        }

        return true;
    }

    void RayTracingValidator::setMargins(const float& widthMargin, const float& lengthMargin)
    {
        _doubledWidthMargin_m = 2*widthMargin;
        _doubledLengthMargin_m = 2*lengthMargin;
    }   






    void RayTracingValidator::setVehicle(std::vector<VehicleFeature> vehicle)
    {
        _vehicle = vehicle;
    }

    float RayTracingValidator::getMinimumSafeDistance(){ return _minimumSafeDistance; }
}