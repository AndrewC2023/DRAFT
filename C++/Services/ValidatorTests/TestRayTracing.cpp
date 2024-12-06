/*
 * Author: Andrew Campbell
 * Date: 12-06-2024
 */

#include "2D/2DGridManager.hpp"
#include "2D/Validators/RayTracingValidator.hpp"
#include "Util/Config/Config.hpp"

#define VALIDATOR_VISUALIZATION_DEBUG

int main()
{
    using namespace Algorithms::TwoD;

    const std::string configPath = "Config.yml";
    const Algorithms::Configuration::Config config = Algorithms::Configuration::loadConfig(configPath);

    // make the grid
    std::shared_ptr<GridManager2D> grid = std::make_shared<GridManager2D>(0.0, 10.0, 0.0, 10.0, 0.1);

    std::deque<PointXY> path({PointXY(1.0,1.0),PointXY(8.0,8.0)});

    std::vector<VehicleFeature> vehicle({
        {PointXY(0.0,0.0), std::vector<PointXY>({PointXY(0.5, -0.5), PointXY(0.5, 0.5), PointXY(-0.5, 0.5), PointXY(-0.5,0.5)})}
    });

    auto* validator = new RayTracingValidator(config, grid, vehicle);

    delete validator;
    return 0;
}
