/*
 * Author: Andrew Campbell
 * Date: 12-04-2024
 */

#include <2D/2DGridManager.hpp>
#include "Util/Visualization/2DVisualizer.hpp"

#include <iostream>
#include <memory>

int main()
{
    // make the grid
    std::shared_ptr<Algorithms::TwoD::GridManager2D> grid = std::make_shared<Algorithms::TwoD::GridManager2D>(0.0, 10.0, 0.0, 10.0, 0.1);

    // auto obstacle1 = Algorithms::TwoD::StaticObstacle(Algorithms::TwoD::PointXY(5.0,5.0),1.5,1.0,M_PI/4);
    // auto obstacle2 = Algorithms::TwoD::StaticObstacle(std::vector<Algorithms::TwoD::PointXY>({Algorithms::TwoD::PointXY(6.0,3.0),Algorithms::TwoD::PointXY(5.5,4.4),Algorithms::TwoD::PointXY(3.2,2.5)}));


    grid->addKnownObstacle(std::make_unique<Algorithms::TwoD::StaticObstacle>(Algorithms::TwoD::PointXY(5.0,5.0), 3, 1.0, M_PI/4));
    grid->addKnownObstacle(std::make_unique<Algorithms::TwoD::StaticObstacle>(std::vector<Algorithms::TwoD::PointXY>({Algorithms::TwoD::PointXY(6.0,3.0),Algorithms::TwoD::PointXY(5.5,4.4),Algorithms::TwoD::PointXY(2.2,8.5)})));

    auto* visualizer = new Visualization::Visualizer2D();

    float time = 0;
    
    visualizer->plotGrid(grid);
    visualizer->show("Grid population test");

    std::cout << "Press Enter to end the test" << std::endl;
    std::cin.get();

    delete visualizer;

    return 0;
}
