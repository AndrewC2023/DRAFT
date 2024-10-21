/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#include "RRTStar2D.hpp"

namespace Algorithms::TwoD
{
    RRTStar2D::RRTStar2D(Configuration::Config& Config,
                         std::shared_ptr<GridManager> grid,
                         std::shared_ptr<IPathValidator2D> validator):
                         Grid(std::move(grid)),
                         Validator(std::move(validator)),
                         _maxIterations(Config.planners.RRTStar.maxIterations),
                         _maxEdgeLength(Config.planners.RRTStar.maxEdgeLength),
                         _endBias(Config.planners.RRTStar.endBias)
    {
        Tree.clear();
        

    };

    std::deque<PointXY> RRTStar2D::PlanPath(PointXY start, PointXY goal)
    {
        // setup:

        // get our sample domain
        Grid->getGridDomain(xMin, xMax, yMin, yMax); // sets them by reference

        

        // set them as gloabal

        // initialize the tree
        Tree.clear();
        Tree.push_back(RRTStarNode({start, 0.0f, 0, 0}));

        for(int iteration; iteration < _maxIterations; iteration++)
        {
            // sample a new node
            bool successfulSample = false;
            while (successfulSample == false)
            {
                PointXY sample = sampleNewNode();
                // find nearest node TODO make function
                float lowestManhattanDistance = INFINITY;
                int closestTreeIndex = 0;
                for(int i = 0; i < numNodes; i++)
                {
                    float CurrentManhattanDist = std::pow(Tree.at(i).position.x - sample.x, 2) + std::pow(Tree.at(i).position.y - sample.y,2);
                    if(CurrentManhattanDist < lowestManhattanDistance)
                    {
                        closestTreeIndex = i;
                        lowestManhattanDistance = CurrentManhattanDist;
                    }
                }

                steer(sample, Tree.at(closestTreeIndex).position, successfulSample);
                // steer function handles the while loop condition
            }

        }

    }

    PointXY sampleNewNode()
    {
        bool sucessfulSample = false;
        while(sucessfulSample != true)
        {
            // sample a node
            PointXY sample(,);
            // check if its in a safe region
        }
    };

    void steer(PointXY& sampledPoint, PointXY nearestNode, bool& rejectNode)
    {
        
    };

    float RRTStar2D::getPathCost(){ return pathCost; };

} // namespace Algorithm::2D