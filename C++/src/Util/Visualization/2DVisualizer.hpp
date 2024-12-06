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
#include <vector>
#include <utility>
#include <iostream>
#include <string>

namespace Visualization
{
    class Visualizer2D
    {
        public:
            Visualizer2D(std::shared_ptr<Algorithms::TwoD::GridManager2D>& );
            ~Visualizer2D() = default;

            void plotGrid(float);
            void plotPath(const std::deque<Algorithms::TwoD::PointXY>&);
            void plotTree(const std::vector<Algorithms::TwoD::RRTStarNode>&);
            void plotPolygon(const std::vector<Algorithms::TwoD::PointXY>&);

            void show(std::string);

        private:

            const Algorithms::TwoD::PointXY convert2IndexSpace(const Algorithms::TwoD::PointXY point);

            const Algorithms::TwoD::PointXY convert2PointSpace(const Algorithms::TwoD::PointXY IndexPoint);

            const Algorithms::TwoD::PointXY convert2PointSpace(const Algorithms::TwoD::IndexXY index);

            std::shared_ptr<Algorithms::TwoD::GridManager2D>&_grid;
        
            matplot::figure_handle _figure;

            float xMin;
            float xMax;
            float yMin;
            float yMax;
    };
}

#endif // VISUALIZER2D_H