/*
 * Author: Andrew Campbell
 * Date: 06-09-2024
 */

#include "Config.hpp"

using namespace Algorithms::Configuration;

// store the line of the function that we are currently calling
decltype ( __LINE__ ) calledLine;

void operator>>(const YAML::Node& node, Room2D& room2D)
{
    calledLine = __LINE__;

    room2D.xMin = node["xMin"].as<float>();
    room2D.xMax = node["xMax"].as<float>();
    room2D.yMin = node["yMin"].as<float>();
    room2D.yMax = node["xMax"].as<float>();

    // load in the obstacles
    const auto obstacles = node["obstacles"].as<std::vector<std::vector<float>>>();

    for( const auto& obstacle : obstacles)
    {
        Obstacle2DStruct obstacleConfig;
        obstacleConfig.xPosition = obstacle.at(0);
        obstacleConfig.yPosition = obstacle.at(1);

        obstacleConfig.length = obstacle.at(2);
        obstacleConfig.width = obstacle.at(3);

        obstacleConfig.yaw = obstacle.at(4);

        room2D.obstacles.push_back(obstacleConfig);
    }
}

void operator>>(const YAML::Node& node, Room3D& room3D)
{
    calledLine = __LINE__;

    room3D.xMin = node["xMin"].as<float>();
    room3D.xMax = node["xMax"].as<float>();
    room3D.yMin = node["yMin"].as<float>();
    room3D.yMax = node["xMax"].as<float>();
    room3D.zMin = node["zMin"].as<float>();
    room3D.zMax = node["zMax"].as<float>();

    // load in the obstacles
    const auto obstacles = node["obstacles"].as<std::vector<std::vector<float>>>();

    for( const auto& obstacle : obstacles)
    {
        Obstacle3DStruct obstacleConfig;
        obstacleConfig.xPosition = obstacle.at(0);
        obstacleConfig.yPosition = obstacle.at(1);
        obstacleConfig.zPosition = obstacle.at(2);

        obstacleConfig.length = obstacle.at(3);
        obstacleConfig.width = obstacle.at(4);
        obstacleConfig.height = obstacle.at(5);

        obstacleConfig.yaw = obstacle.at(6);
        obstacleConfig.pitch = obstacle.at(7);
        obstacleConfig.roll = obstacle.at(8);

        room3D.obstacles.push_back(obstacleConfig);
    }
}

void operator>>(const YAML::Node& node, TestSpace2D& testSpace)
{
    calledLine = __LINE__;
    testSpace.roomName = node["RoomName"].as<std::string>();
    node[testSpace.roomName] >> testSpace.room;
}

void operator>>(const YAML::Node& node, TestSpace3D& testSpace)
{
    calledLine = __LINE__;
    testSpace.roomName = node["RoomName"].as<std::string>();
    node[testSpace.roomName] >> testSpace.room;
}

void operator>>(const YAML::Node& node, Grid2D& grid)
{
    calledLine = __LINE__;
    grid.cellSize = node["cellSize"].as<float>();
    grid.updateFrequency = node["updateFrequency"].as<int>();
    node["TestSpace"] >> grid.testSpace;
}

void operator>>(const YAML::Node& node, Grid3D& grid)
{
    calledLine = __LINE__;
    grid.cellSize = node["cellSize"].as<float>();
    grid.updateFrequency = node["updateFrequency"].as<int>();
    node["TestSpace"] >> grid.testSpace;
}

void operator>>(const YAML::Node& node, GridConfig grid)
{
    calledLine = __LINE__;
    node["TwoD"] >> grid.twoD;
    node["ThreeD"] >> grid.threeD;
}




void operator>>(const YAML::Node& node, RRTStarConfig& RRTStar)
{
    calledLine = __LINE__;
    RRTStar.maxEdgeLength = node["maxEdgeLength"].as<float>();
    RRTStar.maxIterations = node["maxIterations"].as<int>();
    RRTStar.endBias = node["endBias"].as<float>();
}

void operator>>(const YAML::Node& node, EP_RRTStarConfig& EP_RRTStar)
{
    calledLine = __LINE__;
    EP_RRTStar.maxSamplingRange = node["maxSamplingRange"].as<float>();
}

void operator>>(const YAML::Node& node, LQR_RRTStarConfig LQR_RRTStar)
{
    calledLine = __LINE__;
}

void operator>>(const YAML::Node& node, CL_RRTStarConfig CL_RRTStar)
{
    calledLine = __LINE__;
}

void operator>>(const YAML::Node& node, RRTSharpConfig RRTSharp)
{
    calledLine = __LINE__;
}

void operator>>(const YAML::Node& node, RH_ELQRConfig RH_ELQR)
{
    calledLine = __LINE__;
}

void operator>>(const YAML::Node& node, AstarConfig Astar)
{
    calledLine = __LINE__;
    Astar.distanceCostFactor = node["distanceCostFactor"].as<float>();
    Astar.heuristicCostFactor = node["heuristicCostFactor"].as<float>();
    Astar.isInvalidPenalty = node["isInvalidPenalty"].as<float>();
}

void operator>>(const YAML::Node& node, PlannersConfig planners)
{
    calledLine = __LINE__;
    node["RRTStar"] >> planners.RRTStar;
    node["EP_RRTStar"] >> planners.EP_RRTStar;
    node["LQR_RRTStar"] >> planners.LQR_RRTStar;
    node["CL_RRTStar"] >> planners.CL_RRTStar;
    node["RRTSharp"] >> planners.RRTSharp;
    node["RH_ELQR"] >> planners.RH_ELQR;
    node["AStar"] >> planners.Astar;
}

void operator>>(const YAML::Node& node, Config config)
{
    calledLine = __LINE__;

    node["Grid"] >> config.grid;
    node["Planners"] >> config.planners;
}

Config Algorithms::Configuration::loadConfig(const std::string &configPath) {
    calledLine = __LINE__;
    std::fstream configStream(configPath, std::fstream::in);
    if (!configStream.is_open()) {
        throw std::runtime_error("Unable to open file: " + configPath);
    }

    return parseConfig(configStream);
}

Config Algorithms::Configuration::parseConfig(std::istream &configFile) {
    Config config;

    try {
        YAML::Node doc = YAML::Load(configFile);
        if (doc.size() == 0) {
            throw std::runtime_error("Error reading config file: No configuration file.");
        }
        doc >> config;
    } catch (std::runtime_error &e) {
        throw std::runtime_error("Error reading config file: " + std::string(e.what()) + ". Was in function on line " +
                                 std::to_string(calledLine));
    }

    return config;
}