/*
 *Author: Andrew Campbell
 * Date: 11-07-2024
 */

#ifndef VISUALIZER2D_H
#define VISUALIZER2D_H

// Custom Classes
#include "2D/2DGridManager.hpp"



// STL
#include <matplot.h>
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
        void plotPath(std::deque<>);
    };
}

#endif // VISUALIZER2D_H