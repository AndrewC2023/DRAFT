/*
 * Author: Andrew Campbell
 * Date: 06-09-2024
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

// YAML
#include "yaml-cpp/yaml.h"

// STL
#include <iostream>
#include <istream>
#include <fstream>
#include <string>
#include <vector>

namespace Algorithms::Configuration
{
    // All the following are the config data structures
    
    // Region Grid
        typedef struct obstacle2D{
            float xPosition;
            float yPosition;

            float length;
            float width;

            float yaw;
        } Obstacle2DStruct;

        typedef struct obstacle3D{
            float xPosition;
            float yPosition;
            float zPosition;

            float length;
            float width;
            float height;

            float yaw;
            float pitch;
            float roll;
        } Obstacle3DStruct;

        typedef struct room2D{
            float xMin;
            float xMax;
            float yMin;
            float yMax;
            std::vector<Obstacle2DStruct> obstacles;
        } Room2D;

        typedef struct room3D{
            float xMin;
            float xMax;
            float yMin;
            float yMax;
            float zMin;
            float zMax;
            std::vector<Obstacle3DStruct> obstacles;
        } Room3D;

        typedef struct testSpace2D{
            std::string roomName;
            Room2D room;
        } TestSpace2D;

        typedef struct testSpace3D{
            std::string roomName;
            Room3D room;
        } TestSpace3D;

        typedef struct grid2D{
            int updateFrequency;
            float cellSize;
            TestSpace2D testSpace;
        } Grid2D;

        typedef struct grid3D{
            int updateFrequency;
            float cellSize;
            TestSpace3D testSpace;
        } Grid3D;

        typedef struct grid{
            Grid2D twoD;
            Grid3D threeD;
        } GridConfig;


        // Grid, also contains the testing setups
    // End Region


    // Region Planners
        typedef struct rrtstar{
            float maxEdgeLength;
            int maxIterations;
            float endBias;
        } RRTStarConfig;

        typedef struct ep_rrtstar{
            float maxSamplingRange;
        } EP_RRTStarConfig;

        typedef struct lqr_rrtstar{

        } LQR_RRTStarConfig;

        typedef struct cl_rrtstar{

        } CL_RRTStarConfig;

        typedef struct rrtsharp{

        } RRTSharpConfig;

        typedef struct rh_elqr{

        } RH_ELQRConfig;

        typedef struct astar{
            float distanceCostFactor;
            float heuristicCostFactor;
            float isInvalidPenalty;
        } AstarConfig;
 
        typedef struct planners{
            RRTStarConfig RRTStar;
            EP_RRTStarConfig EP_RRTStar;
            LQR_RRTStarConfig LQR_RRTStar;
            CL_RRTStarConfig CL_RRTStar;
            RRTSharpConfig RRTSharp;
            RH_ELQRConfig RH_ELQR;
            AstarConfig Astar;
        } PlannersConfig;
    // End Region


    // Config structure
    typedef struct config{
        GridConfig grid;
        PlannersConfig planners;
    } Config;

    // The two helpers to load and parse the config
    Config loadConfig(const std::string& configPath);

    Config parseConfig(std::istream& configFile);
}

#endif // CONFIG_HPP