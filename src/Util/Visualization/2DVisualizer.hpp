/*
 *Author: Andrew Campbell
 * Date: 11-07-2024
 */

#ifndef VISUALIZER2D_H
#define VISUALIZER2D_H

// Custom Classes
#include "2D/2DGridManager.hpp"
#include "2D/2DHybridGridManager.hpp"

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
    /**
     * @class Visualizer2D
     * @brief A class for visualizing 2D grids, paths, trees, and polygons.
     * 
     * This class provides functionalities to plot and visualize various 2D structures
     * such as grids, paths, trees, and polygons using the matplot library. It also 
     * includes methods to convert between different coordinate spaces.
     * 
     * @note This class requires a shared pointer to a GridManager2D object for initialization.
     *
     * This class provides methods to plot various 2D structures such as grids, paths, trees, and polygons.
     * It also provides functionality to display the plotted figures.
     */
    class Visualizer2D
    {
        public:
            Visualizer2D();
            ~Visualizer2D() = default;

            void plotGrid(std::shared_ptr<Algorithms::TwoD::GridManager2D>&);
            void plotGrid(std::shared_ptr<Algorithms::TwoD::HybridGridManager2D>&, float);
            void plotPath(const std::deque<Algorithms::TwoD::PointXY>&);
            void plotTree(const std::vector<Algorithms::TwoD::RRTStarNode>&);
            
            void plotPolygon(const std::vector<Algorithms::TwoD::PointXY>&);

            void show(std::string);
            void newFigure();
            void setFigure(int);

        private:

            const Algorithms::TwoD::PointXY convert2IndexSpace(const Algorithms::TwoD::PointXY point);

            const Algorithms::TwoD::PointXY convert2PointSpace(const Algorithms::TwoD::PointXY IndexPoint);

            const Algorithms::TwoD::PointXY convert2PointSpace(const Algorithms::TwoD::IndexXY index);
        
            std::vector<matplot::figure_handle> _figure;

            float xMin;
            float xMax;
            float yMin;
            float yMax;

            float _cellSize;
    };
}

#endif // VISUALIZER2D_H