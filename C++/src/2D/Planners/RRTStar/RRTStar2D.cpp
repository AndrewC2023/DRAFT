/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#include "RRTStar2D.hpp"

namespace Algorithms::TwoD
{
    RRTStar2D::RRTStar2D(Configuration::Config& Config,
                         std::shared_ptr<GridManager> Grid,
                         std::shared_ptr<IPathValidator> Validator):
                         _Grid(std::move(Grid)),
                         _Validator(std::move(Validator)),
                         _maxIterations(Config.planners.RRTStar.maxIterations),
                         _maxEdgeLength(Config.planners.RRTStar.maxEdgeLength),
                         _endBias(Config.planners.RRTStar.endBias)
    {
        Tree.clear();
    }

    RRTStar2D::PlanPath(PointXY start, PointXY goal)
    {
        // initialize the tree
        Tree.clear();
        Tree.push_back(RRTStarNode({start, 0f, 0i, 0i}));

        for(int iteration, iteration < _maxIterations, iteration++)
        {
            // sample a new node
            bool successfulSample = false;
            while (successfulSample == false)
            {
                PointXY sample = sampleNewNode();
                // find nearest node
                float lowestManhattanDistance = INFINITY;
                int closestTreeIndex = 0;
                for(int i = 0; i < numNodes: i++)
                {
                    float CurrentManhattanDist = (Tree.at(i).position.x - sample.x)^2 + (Tree.at(i).position.y - sample.y)^2;
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

            // check if its in a safe region
        }
        
    };

    void steer(PointXY& sampledPoint, PointXY nearestNode, bool rejectNode&)
    {
        
    };

    float RRTStar2D::getPathCost(){ return pathCost };

} // namespace Algorithm::2D