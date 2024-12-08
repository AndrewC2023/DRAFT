/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#include "TimeRRTStar2D.hpp"

namespace Algorithms::TwoD
{
    TimeRRTStar2D::TimeRRTStar2D(const Configuration::Config& Config,
                         std::shared_ptr<GridManager2D> grid,
                         std::shared_ptr<IPathValidator2D> validator):
                         _Grid(std::move(grid)),
                         _Validator(std::move(validator)),
                         _maxIterations(Config.planners.RRTStar.maxIterations),
                         _maxEdgeLength(Config.planners.RRTStar.maxEdgeLength),
                         _endBias(Config.planners.RRTStar.endBias),
                         _steerStepSize(Config.planners.RRTStar.steerStep),
                         _invalidPenalty(Config.planners.RRTStar.invalidPenalty),
                         _randGenX({std::random_device{}()}),
                         _randGenY({std::random_device{}()}),
                         _randGenGoalBias({std::random_device{}()})
    {
        Tree.clear(); // make sure the tree is empty
        _goalBiasDistribution = std::uniform_real_distribution<float>(0,1);
    };


    std::deque<PointXY> TimeRRTStar2D::PlanPath(PointXY start, PointXY goal)
    {

        // TODO: error handling for bogus start and/or goal points
        _goal = goal;
        // setup:
        // get our sample domain
        _Grid->getGridDomain(xMin, xMax, yMin, yMax); // sets them by reference
        _xDistribution = std::uniform_real_distribution<float>(xMin,xMax);
        _yDistribution = std::uniform_real_distribution<float>(yMin,yMax);

        // Empty the tree if this is being called again
        Tree.clear();
        Tree.push_back(RRTStarNode({start, 0.0f, 0, 0}));
        
        numNodes = 1;
        int goalIndex = -1;
        foundEnd = false;

        // std::cout << "Starting RRT* Path Planning" << std::endl;

        for(int iteration = 0; iteration < _maxIterations; iteration++)
        {
            // loop goals
            int closestTreeIndex = 0;
            PointXY sample;

            // sample a new node
            bool successfulSample = false; // this condition stipulates that we must also be able to "steer" the node to a valid location
            while (successfulSample == false)
            {
                sample = sampleNewNode();
                // find nearest node TODO make function
                float lowestManhattanDistance = INFINITY;
                closestTreeIndex = 0;
                for(int i = 0; i < numNodes; i++)
                {
                    float CurrentManhattanDist = std::pow(Tree[i].position.x - sample.x, 2) + std::pow(Tree[i].position.y - sample.y,2);
                    if(CurrentManhattanDist < lowestManhattanDistance)
                    {
                        closestTreeIndex = i;
                        lowestManhattanDistance = CurrentManhattanDist;
                    }
                }

                steer(sample, Tree.at(closestTreeIndex).position, successfulSample);
                // steer function handles the while loop condition
            }

            // debug
            // std::cout << "Sampled Point: " << sample << std::endl;

            // sucessful sample
            float cost = costFunction(sample, Tree.at(closestTreeIndex).position) + Tree.at(closestTreeIndex).cost;
            // add the node to the tree
            Tree.push_back(RRTStarNode({sample, cost, numNodes, closestTreeIndex}));

            if(!foundEnd && sample == goal)
            {
                std::cout << "Found the goal!" << std::endl << "Iteration: " << iteration << std::endl << "refining path" << std::endl;
                foundEnd = true;
                goalIndex = numNodes;
            }

            numNodes++;

            rewire();

            std::cout << "Iteration: " << iteration << std::endl;

        }

        if(!foundEnd) //TODO: real error handling
        {
            throw std::runtime_error("Failed to find a path");
        }

        // find the path
        std::deque<PointXY> path{};
        int currentIndex = goalIndex; // TODO this is wehre the bug is
        while(currentIndex != 0)
        {
            path.push_front(Tree.at(currentIndex).position);
            currentIndex = Tree.at(currentIndex).parentIndex;
            std::cout << "path " << path.front() << std::endl; 
        }

        path.push_front(Tree.at(currentIndex).position); // add the start node
        return path;

    } // PlanPath


    /**
     * @brief Samples a new node in the 2D space for the RRT* algorithm.
     * 
     * This function attempts to sample a new node with a goal bias. If the goal bias
     * condition is met and the goal has not been found yet, it returns the goal node.
     * Otherwise, it samples a node within the defined space until a valid (safe) node
     * is found.
     * 
     * @return PointXY The sampled node, either the goal node or a valid random node.
     */
    PointXY TimeRRTStar2D::sampleNewNode()
    {
        // goal bias, however we need to check if we have already found the goal
        if(_goalBiasDistribution(_randGenGoalBias) < _endBias && !foundEnd)
        {
            // std::cout << "Goal sampled" << std::endl;
            // sucessful sample the goal
            return _goal; // the assumption is that the gaol is clear maybe we need a prestep to validate this

        }
        // else
        bool sucessfulSample = false;
        PointXY sample;
        while(sucessfulSample != true)
        {
            // sample a node
            sample.x = _xDistribution(_randGenX);
            sample.y = _yDistribution(_randGenY);
            // check if its in a safe region
            sucessfulSample = (_Grid->getCell(sample).getState() == State::CLEAR);
        }
        return sample;

    } // sampleNewNode

    void TimeRRTStar2D::steer(PointXY& sampledPoint, PointXY nearestNode, bool& successful)
    {

        successful = false;
        
        // Calculate the unit vector
        PointXY vector = sampledPoint - nearestNode;
        PointXY unit_vector = vector / vector.norm();
        PointXY stepVector = PointXY(unit_vector.x * _steerStepSize, unit_vector.y * _steerStepSize);

        // std::cout << "xStep " << stepVector.x * _steerStepSize << std::endl;
        // std::cout << "yStep " << stepVector.y * _steerStepSize << std::endl;     
        // std::cout << "steerStepSize " << _steerStepSize << std::endl;

        // std::cout << "in steer function" << std::endl;
        // std::cout << "vector to steer along: " << vector << std::endl;
        // std::cout << "unit vector: " << unit_vector << std::endl;
        // std::cout << "step vector: " << stepVector << std::endl << std::endl;    
        // std::cout << "yada pee pee" << nearestNode + (unit_vector * _maxEdgeLength) << std::endl;

        // fast ways to exit to avoid more calls to the validator
        if (vector.norm() < _steerStepSize)
        {
            if(_Validator->validatePathSegment(nearestNode, sampledPoint, 0.0f))
            {
                successful = true;
                return;
            }
            else
            {
                successful = false; // we failed to validate the path
                return;
            }
        }
        
        
        if((sampledPoint - nearestNode).norm() < _maxEdgeLength)
        {
            if(_Validator->validatePathSegment(nearestNode, sampledPoint, 0.0f))
            {
                successful = true;
                return;
            }
            
        }
        
        
        // Loop setup
        PointXY lastPoint = nearestNode;
        PointXY tempPoint = lastPoint + stepVector;

        float travelled = _steerStepSize;
        bool first = true;
        while (!successful)
        {
            travelled += _steerStepSize;

            if (first)
            {
                first = false;
                if(!_Validator->validatePathSegment(nearestNode, tempPoint, 0.0f))
                {
                    successful = false;
                    return;
                }
                // else we hav ethe ability to steer until an obstacle is hit
            }

            if (!_Validator->validatePathSegment(lastPoint, tempPoint, 0.0f))
            {
                // Hit obstacle, exit
                sampledPoint = lastPoint;
                successful = true;
                return;
            }
            else if (travelled >= _maxEdgeLength)
            {
                // Max length Reached
                sampledPoint = tempPoint;
                successful = true;
                return;
            }
            else if (std::sqrt(std::pow(tempPoint.x - sampledPoint.x, 2) + std::pow(tempPoint.y - sampledPoint.y, 2)) < _steerStepSize * 1.1f)
            {
                // Reached point, exit
                sampledPoint = sampledPoint;
                successful = true;
                return;
            }
            else
            {
                lastPoint = tempPoint;
                tempPoint = tempPoint + stepVector;
                
            }

        }

    } // steer

    void TimeRRTStar2D::rewire()
    {
        // we know this ALWAYS happens after the new node is added
        for (int i = 0; i < numNodes - 1; ++i)
        {
            if((Tree[numNodes - 1].position - Tree[i].position).norm() > _maxEdgeLength)
            {
                // we can't rewire
                continue;
            }

            // check if the new node is a better parent
            float newCost = costFunction(Tree[numNodes - 1].position, Tree[i].position) + Tree[numNodes - 1].cost;
            if (newCost < Tree[i].cost)
            {
                // validate the path
                if (!_Validator->validatePathSegment(Tree[i].position, Tree[numNodes - 1].position, 0.0f))
                {
                    // Invalid, we can't rewire
                    continue;
                }
                // rewire the node
                Tree[i].cost = newCost;
                Tree[i].parentIndex = numNodes - 1;
            }
        }
    } // Rewire

    float TimeRRTStar2D::costFunction(const PointXY& sampledPoint, const PointXY& nearestNode)
    {
        float distance = std::sqrt(std::pow(sampledPoint.x - nearestNode.x, 2) + std::pow(sampledPoint.y - nearestNode.y, 2));
        // any other punishements go here
        float cost = distance;
        return cost;
    } // costFunction

    std::vector<RRTStarNode> TimeRRTStar2D::getTree(){ return Tree; };

    float TimeRRTStar2D::getPathCost(){ return pathCost; };

} // namespace Algorithm::2D