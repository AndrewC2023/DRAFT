/*
 * Author: Andrew Campbell
 * Date: 12-06-2024
 */

#include "2D/2DGridManager.hpp"
#include "2D/Validators/RayTracingValidator.hpp"
#include "2D/Planners/RRTStar/RRTStar2D.hpp"
#include "Util/Config/Config.hpp"
#include "Util/Visualization/2DVisualizer.hpp"
#include "Util/CustomTypes/VectorAndPointTypes.hpp"
#include "2D/IPathValidator2D.hpp"

#include <iostream>

int main()
{
    using namespace Algorithms::TwoD;
    
    const std::string configPath = "Config.yml";
    const Algorithms::Configuration::Config config = Algorithms::Configuration::loadConfig(configPath);

    // make the grid
    std::shared_ptr<GridManager2D> grid = std::make_shared<GridManager2D>(0.0, 10.0, 0.0, 10.0, 0.1);

    std::vector<Algorithms::TwoD::VehicleFeature> vehicle({
        {PointXY(0.0,0.0), std::vector<PointXY>({PointXY(0.25, -0.25), PointXY(0.25, 0.25), PointXY(-0.25, 0.25), PointXY(-0.25,0.25)})}
    });
    std::shared_ptr<IPathValidator2D> validator = std::make_shared<RayTracingValidator>(config, grid, vehicle);
    
    std::shared_ptr<RRTStar2D> planner = std::make_shared<RRTStar2D>(config, grid, validator);

    grid->addKnownObstacle(std::make_unique<Algorithms::TwoD::StaticObstacle>(Algorithms::TwoD::PointXY(5.0,5.0), 2, 0.67, -M_PI/4));

    std::deque<PointXY> path = planner->PlanPath(PointXY(1.0,1.0), PointXY(8.0,8.0));
    std::vector<RRTStarNode> tree = planner->getTree();
    
    auto* visualizer = new Visualization::Visualizer2D(grid);

    visualizer->plotGrid(0.0);
    //visualizer->plotPath(path);
    visualizer->plotTree(tree);
    visualizer->show("RRT* Path Planning");
   
    return 0;

}
