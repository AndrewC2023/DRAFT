/*
 * Author: Andrew Campbell
 * Date: 12-02-2024
 */

#include "2D/2DGridManager.hpp"
#include "2D/Validators/RayTracingValidator.hpp"
#include "Util/Config/Config.hpp"

#include <iostream>

int main()
{
    using namespace Algorithms::TwoD;
    
    const std::string configPath = "Config.yml";
    Algorithms::Configuration::Config config = Algorithms::Configuration::loadConfig(configPath);
    config.validators.debug = true; // turn on debug for the validator if not on

    // make the grid
    std::shared_ptr<GridManager2D> grid = std::make_shared<GridManager2D>(0.0, 10.0, 0.0, 10.0, 0.1);
    grid->addKnownObstacle(std::make_unique<Algorithms::TwoD::StaticObstacle>(Algorithms::TwoD::PointXY(7.0,2.0), 2, 0.67, -M_PI/4));

    std::deque<PointXY> path({PointXY(2.0,2.0),PointXY(4.4,3.0)});

    std::vector<VehicleFeature> vehicle({
        {PointXY(0.0,0.0), std::vector<PointXY>({PointXY(0.25, -0.25), PointXY(0.25, 0.25), PointXY(-0.25, 0.25), PointXY(-0.25,0.25)})}
    });

    
    auto* validator = new RayTracingValidator(config, grid, vehicle);
    float prob = 0.0;
    if(validator->validatePath(path, 0.0, prob))
    {
        std::cout << "Path is Valid\n";
    } else{
        std::cout << "Path is Invalid\n";
    }

    delete validator;


    return 0;
}
