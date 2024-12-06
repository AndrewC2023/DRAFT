/*
 * Author: Andrew Campbell
 * Date: 12-05-2024
 */

#include <2D/2DGridManager.hpp>
#include "Util/Visualization/2DVisualizer.hpp"

#include <iostream>
#include <memory>

int main()
{
    // make the grid
    std::shared_ptr<Algorithms::TwoD::GridManager2D> grid = std::make_shared<Algorithms::TwoD::GridManager2D>(0.0, 10.0, 0.0, 10.0, 1.0);

    auto obstacle1 = Algorithms::TwoD::StaticObstacle(Algorithms::TwoD::PointXY(5.0,5.0),1.5,1.0,M_PI/4);

    grid->addKnownObstacle(std::make_unique<Algorithms::TwoD::StaticObstacle>(Algorithms::TwoD::PointXY(5.0,5.0), 1.5, 1.0, M_PI/4));

    auto* visualizer = new Visualization::Visualizer2D(grid);

    visualizer->plotGrid();
    visualizer->show();

    std::cout << "Press Enter to end the test" << std::endl;
    std::cin.get();

    delete visualizer;

    return 0;
}
