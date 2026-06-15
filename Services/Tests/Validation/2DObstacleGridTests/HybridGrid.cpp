/*
 * Author: Andrew Campbell
 * Date: 12-04-2024
 */

#include <2D/2DHybridGridManager.hpp>
#include "Util/Visualization/2DVisualizer.hpp"

#include "2D/Planners/RRTStar/TimeRRTStar2D.hpp"
#include "Util/Config/Config.hpp"
#include "Util/CustomTypes/VectorAndPointTypes.hpp"
#include "2D/IPathValidator2D.hpp"
#include "2D/Validators/RayTracingValidator.hpp"

#include <iostream>
#include <memory>
#include <cmath>

int main()
{
    const std::string configPath = "Config.yml";
    const Algorithms::Configuration::Config config = Algorithms::Configuration::loadConfig(configPath);

    PointXY point(1.0,1.0);

    // make the grid
    std::shared_ptr<Algorithms::TwoD::HybridGridManager2D> grid = std::make_shared<Algorithms::TwoD::HybridGridManager2D>(0.0, 10.0, 0.0, 10.0, 0.1);

    grid->addKnownObstacle(std::make_unique<Algorithms::TwoD::StaticObstacle>(std::vector<Algorithms::TwoD::PointXY>(
                                            {Algorithms::TwoD::PointXY(3.822,4.318),
                                             Algorithms::TwoD::PointXY(3.858,4.182),
                                             Algorithms::TwoD::PointXY(3.979,4.042),
                                             Algorithms::TwoD::PointXY(4.172,3.989),
                                             Algorithms::TwoD::PointXY(4.402,4.07),
                                             Algorithms::TwoD::PointXY(4.637,4.331),
                                             Algorithms::TwoD::PointXY(4.8355,4.6795),
                                             Algorithms::TwoD::PointXY(4.714,4.735),
                                             Algorithms::TwoD::PointXY(4.648,4.907),
                                             Algorithms::TwoD::PointXY(4.6957,5.079),
                                             Algorithms::TwoD::PointXY(4.802,5.152),
                                             Algorithms::TwoD::PointXY(4.8087,5.0148),
                                             Algorithms::TwoD::PointXY(4.885,4.9376),
                                             Algorithms::TwoD::PointXY(4.96,4.94),
                                             Algorithms::TwoD::PointXY(5.224,5.37),
                                             Algorithms::TwoD::PointXY(5.62,5.906),
                                             Algorithms::TwoD::PointXY(5.187,5.9416),
                                             Algorithms::TwoD::PointXY(5.027,6.016),
                                             Algorithms::TwoD::PointXY(4.955,6.164),
                                             Algorithms::TwoD::PointXY(5.001,6.3064),
                                             Algorithms::TwoD::PointXY(5.0665,6.3345),
                                             Algorithms::TwoD::PointXY(5.0695,6.2626),
                                             Algorithms::TwoD::PointXY(5.131,6.1835),
                                             Algorithms::TwoD::PointXY(5.2947,6.1198),
                                             Algorithms::TwoD::PointXY(5.5716,6.1053),
                                             Algorithms::TwoD::PointXY(5.8317,6.1487),
                                             Algorithms::TwoD::PointXY(6.119,6.346),
                                             Algorithms::TwoD::PointXY(6.3095,6.2534),
                                             Algorithms::TwoD::PointXY(6.12,5.84),
                                             Algorithms::TwoD::PointXY(5.9355,5.095),
                                             Algorithms::TwoD::PointXY(6.114,5.098),
                                             Algorithms::TwoD::PointXY(6.22,5),
                                             Algorithms::TwoD::PointXY(6.205,4.794),
                                             Algorithms::TwoD::PointXY(6.093,4.708),
                                             Algorithms::TwoD::PointXY(6.04,4.805),
                                             Algorithms::TwoD::PointXY(5.906,4.788),
                                             Algorithms::TwoD::PointXY(5.874,4.506),
                                             Algorithms::TwoD::PointXY(5.883,4.13),
                                             Algorithms::TwoD::PointXY(5.962,4.028),
                                             Algorithms::TwoD::PointXY(6.084,4.008),
                                             Algorithms::TwoD::PointXY(6.3535,4.101),
                                             Algorithms::TwoD::PointXY(6.171,3.81),
                                             Algorithms::TwoD::PointXY(6.33,3.953),
                                             Algorithms::TwoD::PointXY(5.937,3.744),
                                             Algorithms::TwoD::PointXY(5.7,3.823),
                                             Algorithms::TwoD::PointXY(5.57,4.111),
                                             Algorithms::TwoD::PointXY(5.597,4.741),
                                             Algorithms::TwoD::PointXY(5.15,4.68),
                                             Algorithms::TwoD::PointXY(4.8,4.1),
                                             Algorithms::TwoD::PointXY(4.436,3.794),
                                             Algorithms::TwoD::PointXY(4.067,3.739),
                                             Algorithms::TwoD::PointXY(3.815,3.842),
                                             Algorithms::TwoD::PointXY(3.69,4.009),
                                             Algorithms::TwoD::PointXY(3.648,4.209),
                                             Algorithms::TwoD::PointXY(3.711,4.391),
                                             Algorithms::TwoD::PointXY(3.811,4.48),
                                             Algorithms::TwoD::PointXY(3.86,4.46)
                                             })));

    std::vector<Algorithms::TwoD::PointXY> corners = {Algorithms::TwoD::PointXY(0.75,-0.4),
                                                      Algorithms::TwoD::PointXY(0.75,0.4),
                                                      Algorithms::TwoD::PointXY(0.0,0.65),
                                                      Algorithms::TwoD::PointXY(-0.75,0.4),
                                                      Algorithms::TwoD::PointXY(-0.75,-0.4),
                                                      Algorithms::TwoD::PointXY(0.0,-0.65)};
    
    Algorithms::TwoD::StateXYT state(2.0,2.0,M_PI/2.0f);

    grid->addKnownObstacle(std::make_unique<Algorithms::TwoD::DynamicUncertainObstacle>(corners, state, 0.0, 0.2f, 0.1f, 1.0f * M_PI /180.0f, 0.15f));

    std::vector<Algorithms::TwoD::VehicleFeature> vehicle({
        {PointXY(0.0,0.0), std::vector<PointXY>({PointXY(0.25, -0.25), PointXY(0.25, 0.25), PointXY(-0.25, 0.25), PointXY(-0.25,0.25)})}
    });
    std::shared_ptr<Algorithms::TwoD::IPathValidator2D> validator = std::make_shared<Algorithms::TwoD::RayTracingValidator>(config, grid, vehicle);

    std::shared_ptr<Algorithms::TwoD::TimeRRTStar2D> planner = std::make_shared<Algorithms::TwoD::TimeRRTStar2D>(config, grid, validator);
    PointXY start(2.0,9.0);
    std::deque<PointXY> path = planner->PlanPath(PointXY(2.0,9.0), PointXY(3.0,1.5));
    std::vector<RRTStarNode> tree = planner->getTree();

    auto* visualizer = new Visualization::Visualizer2D();

    float time = 0;
    
    visualizer->plotGrid(grid,time);
    visualizer->show("Grid Occupancy Probability, time = 0 seconds");

    time = 5;
    visualizer->newFigure();
    visualizer->plotGrid(grid,time);
    visualizer->show("Grid Occupancy Probability, time = 5 seconds");

    time = 10;
    visualizer->newFigure();
    visualizer->plotGrid(grid,time);
    visualizer->show("Grid Occupancy Probability, time = 10 seconds");

    time = 15;
    visualizer->newFigure();
    visualizer->plotGrid(grid,time);
    visualizer->show("Grid Occupancy Probability, time = 15 seconds");

    time = 10;
    visualizer->newFigure();
    visualizer->plotGrid(grid,time);
    visualizer->plotTree(tree);
    visualizer->plotPath(path);
    std::vector<PointXY> polygon;
    for(auto& point : vehicle[0].featurePolygon)
    {
        polygon.push_back(point + start);
    }
    visualizer->plotPolygon(polygon);

    visualizer->show("Planned Path, Showing Probability at time = 10 seconds");



    std::cout << "Press Enter to end the test" << std::endl;
    std::cin.get();

    delete visualizer;

    return 0;
}
