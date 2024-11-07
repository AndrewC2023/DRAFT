/*
 *Author: Andrew Campbell
 * Date: 11-07-2024
 */

#ifndef VISUALIZER2D_H
#define VISUALIZER2D_H

namespace Algorithms::TwoD::Visualization
{
    class Visualizer2D
    {
        public:
        Visualizer2D();
        ~Visualizer2D() = default;

        void plotGrid();
        void plotPath();
    };
}

#endif // VISUALIZER2D_H