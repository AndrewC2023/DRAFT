/*
 *Author: Andrew Campbell
 * Date: 11-07-2024
 */

#ifndef VISUALIZER2D_H
#define VISUALIZER2D_H

// Custom Classes
#include "2D/2DGridManager.hpp"

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"
#include "Util/CustomTypes/PlanningNodes2D.hpp"

// STL
#include <matplot/matplot.h>
#include <memory>
#include <deque>

namespace Visualization
{
    class Visualizer2D
    {
        public:
        Visualizer2D();
        ~Visualizer2D() = default;

        void plotGrid(std::shared_ptr<Algorithms::TwoD::GridManager2D>);
        void plotPath(std::deque<Algorithms::TwoD::PointXY>);
        void plotTree(std::vector<RRTStarNode>);
    };
}

#endif // VISUALIZER2D_H