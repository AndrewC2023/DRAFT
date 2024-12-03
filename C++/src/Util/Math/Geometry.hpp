/*
 * Author: Andrew Campbell
 * Date: 10-16-2024
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"

// dependancies
#include "BasicMath.hpp"

// boost
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>

// STL
#include <vector>
#include <algorithm>
#include <utility>

using namespace Algorithms::TwoD;
using namespace Algorithms::ThreeD;

namespace Math::Geometry
{

    /**  Function that returns true if the polygons formed by the points in a and b intersect one another
     *   i.e. a is a set of 4 points that form a square, and b is a set of 3 points that form a triangle
     *
     * @param a vector of points x/y that form a polygon
     * @param b vector of points x/y that form a polygon
     * 
     * @note polygons a & b should be defined in the rotational order in which the points define the polygon notably the order in which the line of the line integral of the polygon would encounter the points/corners
     * 
     * @return true if the polygons intersect
     */
    [[gnu::hot, maybe_unused, nodiscard]] static bool polygonsIntersect(const std::vector<PointXY>& a, const std::vector<PointXY>& b)
    {
        if(a.empty() || b.empty())
            return false;

        typedef boost::geometry::model::d2::point_xy<float> point_2d;
        typedef boost::geometry::model::polygon<point_2d> polygon_2d;

        // Cast provided polygon to funky boost polygon types
        polygon_2d poly_a;
        polygon_2d poly_b;

        std::vector<point_2d> points_a;
        std::transform(std::begin(a), std::end(a), std::back_inserter(points_a), [](const PointXY& point)
        {
            return point_2d(point.x, point.y);
        });
        points_a.push_back(points_a[0]);
        boost::geometry::assign_points(poly_a, points_a);

        std::vector<point_2d> points_b;
        std::transform(std::begin(b), std::end(b), std::back_inserter(points_b), [](const PointXY& point)
        {
            return point_2d(point.x, point.y);
        });
        points_b.push_back(points_b[0]);
        boost::geometry::assign_points(poly_b, points_b);

        // Fix any issues with the geometries so boost is happy
        boost::geometry::correct(poly_a);
        boost::geometry::correct(poly_b);

        // Results will hold all intersections created by the polygon, in a deque. We really only need to know whether it is 0 or not, but maybe we can use this in the future for something else
        std::deque<point_2d> results;
        boost::geometry::intersection(poly_a, poly_b, results);

        // Boost geometry seems to not like shapes contained within other shapes. The above code will account for intersections where the edges cross, but if one shape is contained within another, it
        // will not be updated. This checks if one shape is contained entirely within another
        return !results.empty() || boost::geometry::within(poly_a, poly_b) || boost::geometry::within(poly_b, poly_a);
    }
    
    /** Function for getting the intersect point of two lines
     * @param line1 the first line fed as two points
     * @param line2 the second line fed as two points
     * 
     * @return a pointXY of where the intersection is
    */
    [[gnu::hot, maybe_unused, nodiscard]] static inline PointXY linesIntersection(const std::pair<PointXY,PointXY>& line1, const std::pair<PointXY,PointXY>& line2)
    {
        // TODO: can be right when the lines are fed in one order over the other?????
        /* 
           This is a Linear equation Ax = b
           It is also exclusively goin to be 2x2 for A and 2x1 for b 
                combines to a 2x3 matrix of the form:
                 [-1, a, | b]
                 [-1, c, | d]
           becuase it will always be this way, it is setup to do the exact calculations for two lines, this is for speed
           - Andrew C
        */
        PointXY intersect;
        if(line1.first.x == line1.second.x && line2.first.x == line2.second.x)
        {
            // parallel and vertical
            return PointXY(0.0,0.0);
        }
        else if(line1.first.y == line1.second.y && line2.first.y == line2.second.y)
        {
            // parallel and horizontal
            return PointXY(0.0,0.0);
        }
        else if(line1.first.y == line1.second.y)
        {
            float c = (line2.second.x - line2.first.x)/(line2.second.y - line2.first.y);
            float d = (line2.first.y * c) - line2.first.x;
            intersect = PointXY( (c * line1.first.y) - d, line1.first.y);
        }
        else if(line2.first.y == line2.second.y)
        {
            float a = (line1.second.x - line1.first.x)/(line1.second.y - line1.first.y);
            float b = (line1.first.y * a) - line1.first.x;
            intersect = PointXY( (a * line2.first.y) - b, line2.first.y);
        }
        else if((line1.second.y - line1.first.y)/(line1.second.x - line1.first.x) == (line2.second.y - line2.first.y)/(line2.second.x - line2.first.x))
        {
            // parallel
            return PointXY(0.0,0.0);
        }
        else
        {
            // not in edge case
            float a = (line1.second.x - line1.first.x)/(line1.second.y - line1.first.y);
            float b = (line1.first.y * a) - line1.first.x;
            float c = (line2.second.x - line2.first.x)/(line2.second.y - line2.first.y);
            float d = (line2.first.y * c) - line2.first.x;
            intersect = PointXY((a*((d - b)/(c - a)) - b) , ((d - b)/(c - a)));
        }
        return intersect;
    }

    /** Determines if two finite lines intersect eachother 
     *  returns true of two lines intersect within the ranges they are provided in
     * @param line1 the first line fed as two points
     * @param line2 the second line fed as two points
     * 
     * @return true if the lines intersect in their ranges, false if not
    */
    [[gnu::hot, maybe_unused, nodiscard]] static inline bool doTwoFiniteLinesIntersect(const std::pair<PointXY,PointXY>& line1, const std::pair<PointXY,PointXY>& line2)
    {
        /* 
           This is a Linear equation Ax = b
           It is also exclusively goin to be 2x2 for A and 2x1 for b 
                combines to a 2x3 matrix of the form:
                 [-1, a, | b]
                 [-1, c, | d]

                 [x, y]'
           becuase it will always be this way, it is setup to do the exact calculations for two lines, this is for speed
           - Andrew C
        */
        
        // check for if either line is horizontal
        PointXY intersect;
        if(line1.first.x == line1.second.x && line2.first.x == line2.second.x)
        {
            // parallel and vertical
            return false;
        }
        else if(line1.first.y == line1.second.y && line2.first.y == line2.second.y)
        {
            // parallel and horizontal
            return false;
        }
        else if(line1.first.y == line1.second.y)
        {
            float c = (line2.second.x - line2.first.x)/(line2.second.y - line2.first.y);
            float d = (line2.first.y * c) - line2.first.x;
            intersect = PointXY( (c * line1.first.y) - d, line1.first.y);
        }
        else if(line2.first.y == line2.second.y)
        {
            float a = (line1.second.x - line1.first.x)/(line1.second.y - line1.first.y);
            float b = (line1.first.y * a) - line1.first.x;
            intersect = PointXY( (a * line2.first.y) - b, line2.first.y);
        }
        else if((line1.second.y - line1.first.y)/(line1.second.x - line1.first.x) == (line2.second.y - line2.first.y)/(line2.second.x - line2.first.x))
        {
            // parallel
            return false;
        }
        else
        {
            // not in edge case
            float a = (line1.second.x - line1.first.x)/(line1.second.y - line1.first.y);
            float b = (line1.first.y * a) - line1.first.x;
            float c = (line2.second.x - line2.first.x)/(line2.second.y - line2.first.y);
            float d = (line2.first.y * c) - line2.first.x;
            intersect = PointXY((a*((d - b)/(c - a)) - b) , ((d - b)/(c - a)));
        }
        // check ranges
        bool x1Range = false;
        bool y1Range = false;
        bool x2range = false;
        bool y2range = false;
        if(intersect.x >= std::min(line1.first.x, line1.second.x) && intersect.x <= std::max(line1.first.x, line1.second.x))
            x1Range = true;
        if(intersect.y >= std::min(line1.first.y, line1.second.y) && intersect.y <= std::max(line1.first.y, line1.second.y))
            y1Range = true;
        if(intersect.x >= std::min(line2.first.x, line2.second.x) && intersect.x <= std::max(line2.first.x, line2.second.x))
            x2range = true;
        if(intersect.y >= std::min(line2.first.y, line2.second.y) && intersect.y <= std::max(line2.first.y, line2.second.y))
            y2range = true;
        // only have to check one of the lines, theres a clean proof... I don't have it here

        if(x1Range && y1Range && x2range && y2range)
        {
            return true;
        }else{
            return false;
        }
    }

    /** Helper function to rotate a set of points that form a rectangle around a center point. This has many uses, especially in
     *  navigation. The original use of this function is to generate the points that form the robot's perimeter, using the robot's
     *  pose (x/y/theta) and the robot dimensions.
     * 
     *  @param x the center x position, in meters
     *  @param y the center y position, in meters
     *  @param theta the yaw in radians
     *  @param width_m the width of the object, the y axis with a yaw of 0
     *  @param length_m the length of the object, the x axis with a yaw of 0
     *  
     *  @return a list of point objects that form the outline of the object
     */
    [[maybe_unused, nodiscard]] static std::vector<PointXY> generateRectangularOutline(const PointXY center,
                                                                             const float length,
                                                                             const float width,
                                                                             const float orientation_Rads)
    {
            // make unit vectors:
            Util::CustomTypes::Vectors::Vector2f e1(std::cos(orientation_Rads), std::sin(orientation_Rads));
            Util::CustomTypes::Vectors::Vector2f e2(std::cos(orientation_Rads + M_PI/2), std::sin(orientation_Rads + M_PI/2));

            float hL = length/2;
            float hW = width/2;

            std::vector<PointXY> corners;
            corners.push_back(center + e1*hL - e2*hW);
            corners.push_back(center + e1*hL + e2*hW);
            corners.push_back(center - e1*hL + e2*hW);
            corners.push_back(center - e1*hL - e2*hW);
            return corners;
    }

    /** Helper to generate a downsampled version of a circle, using the circle's center position and radius
     *  The original use of this function was for rendering obstacles, as it must be rendered as triangles
     */
    [[maybe_unused, nodiscard]] static std::vector<PointXY> generateCircularOutline(const float x,
                                                                                    const float y,
                                                                                    const float radius,
                                                                                    const std::size_t numberOfPoints)
    {
        std::vector<PointXY> obstacleShape;
        double radiansPerPoint = static_cast<double>(2*M_PI) / static_cast<double>(numberOfPoints);

        // Each obstacle is a circle-ish, generate a set of points that make up the outline
        for(std::size_t i = 0; i < numberOfPoints; i++)
        {
            const double angle = static_cast<double>(i) * radiansPerPoint;
            const PointXY point = { static_cast<float>(x + radius * std::cos(angle)), static_cast<float>(y + radius * std::sin(angle)) };
            obstacleShape.push_back(point);
        }

        return obstacleShape;
    }

    /** Helper to get the distance from a point to a line, as a vector. Make sure to take the absolute value when using if you don't
     *  care about which side the point is on
     * 
     *  works based of the perpendicular vector component using cross product
     */
    [[maybe_unused, nodiscard]] static inline float distanceFromPointToLine(const PointXY& lineStart, const PointXY& lineEnd, const PointXY& point)
    {
       auto lineDelta = lineEnd - lineStart;
       auto pointDelta = point - lineStart;

       // Magnitude of cross product of point distance and line length is the area of a parallelogram
       float area = std::abs(pointDelta.x * lineDelta.y - pointDelta.y * lineDelta.x);

       // Distance to line is the height of the parallelogram, so area / base = height = distance to point
       return static_cast<float>(area / lineDelta.norm());
    }

    /** Returns true if the provided point is inside the provided polygon outline,
     *  using a ray-casting algorithm. This flips the value of inPolygon back and
     *  forth every time a wall is crossed, which means the number will be odd (true)
     *  if the point lies inside the polygon
     */
    [[gnu::hot, maybe_unused, nodiscard]] static inline bool isPointInsidePolygon(const PointXY& point, const std::vector<PointXY>& polygon)
    {
        bool inPolygon = false;
        for(std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
        {
            if(((polygon[i].y > point.y) != (polygon[j].y > point.y)) && (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
                inPolygon = !inPolygon;
        }
        return inPolygon;
    }
    
}

#endif // GEOMETRY_H