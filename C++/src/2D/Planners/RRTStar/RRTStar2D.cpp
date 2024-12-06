/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#include "RRTStar2D.hpp"

namespace Algorithms::TwoD
{
    RRTStar2D::RRTStar2D(const Configuration::Config& Config,
                         std::shared_ptr<GridManager2D> grid,
                         std::shared_ptr<IPathValidator2D> validator):
                         _Grid(std::move(grid)),
                         _Validator(std::move(validator)),
                         _maxIterations(Config.planners.RRTStar.maxIterations),
                         _maxEdgeLength(Config.planners.RRTStar.maxEdgeLength),
                         _endBias(Config.planners.RRTStar.endBias),
                         _randGenX({std::random_device{}()}),
                         _randGenY({std::random_device{}()}),
                         _randGenGoalBias({std::random_device{}()})
    {
        Tree.clear(); // make sure the tree is empty
        _goalBiasDistribution = std::uniform_real_distribution<float>(0,1);
    };

    std::deque<PointXY> RRTStar2D::PlanPath(PointXY start, PointXY goal)
    {

        // TODO: error handling for bogus start and/or goal points
        _goal = goal;
        // setup:
        // get our sample domain
        _Grid->getGridDomain(xMin, xMax, yMin, yMax); // sets them by reference
        _xDistribution = std::uniform_real_distribution<float>(xMin,xMax);
        _yDistribution = std::uniform_real_distribution<float>(yMin,yMax);

        // Emty the tree if this is being called again
        Tree.clear();
        Tree.push_back(RRTStarNode({start, 0.0f, 0, 0}));
        numNodes = 1;

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

    PointXY RRTStar2D::sampleNewNode()
    {
        // goal bias
        if(_goalBiasDistribution(_randGenGoalBias) < _endBias)
        {
            // sucessful sample the goal
            return _goal; // the assumption is that the gaol is clear maybe we need a prestep to validate this
        }
        // else
        bool sucessfulSample = false;
        while(sucessfulSample != true)
        {
            // sample a node
            PointXY sample(_xDistribution(_randGenX),_yDistribution(_randGenY));
            // check if its in a safe region

        }
    };

    void RRTStar2D::steer(PointXY& sampledPoint, PointXY nearestNode, bool& successful)
    {
        
    };

    float RRTStar2D::getPathCost(){ return pathCost; };

} // namespace Algorithm::2D