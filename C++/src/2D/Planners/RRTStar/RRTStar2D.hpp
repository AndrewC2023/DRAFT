/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#ifndef RRTSTAR2D_H
#define RRTSTAR2D_H

// base class
#include "../../IPathPlanner2D.hpp"

// Dependent Classes
#include "../../2DGridManager.hpp"
#include "../../IPathValidator2D.hpp"

// Config
#include "Util/Config/Config.hpp"

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"
#include "Util/CustomTypes/PlanningNodes2D.hpp"

// STL
#include <vector>
#include <memory>
#include <cmath>
#include <random>

namespace Algorithms::TwoD
{
    /**
     * @class RRTStar2D
     * @brief Implements the RRT* (Rapidly-exploring Random Tree Star) algorithm for 2D path planning.
     * 
     * This class provides the functionality to plan a path in a 2D space using the RRT* algorithm.
     * It inherits from the IPathPlanner2D interface and overrides its methods to provide specific
     * implementations for path planning, path cost calculation, and other necessary operations.
     * 
     * The RRT* algorithm is an extension of the RRT algorithm that improves the quality of the path
     * by rewiring the tree to allow for the convergence towards optimal paths. This class maintains
     * a tree of nodes, samples new points in the space, steers towards them, and rewires the tree to
     * optimize the path.
     * 
     * @note This class uses random number generators for sampling points in the space and for goal biasing.
     * 
     * @param Configuration::Config& Configuration settings for the planner.
     * @param std::shared_ptr<GridManager2D> Shared pointer to the grid manager.
     * @param std::shared_ptr<IPathValidator2D> Shared pointer to the path validator.
     */
    class RRTStar2D : IPathPlanner2D
    {
        public:
            RRTStar2D(const Configuration::Config&, 
                      std::shared_ptr<GridManager2D>,
                      std::shared_ptr<IPathValidator2D>);

            ~RRTStar2D() = default;

            std::deque<PointXY> PlanPath(PointXY start, PointXY goal) override;
            std::vector<RRTStarNode> getTree();
            float getPathCost() override;

        private:
            // The tree of nodes
            std::vector<RRTStarNode> Tree;
            float pathCost;

            int numNodes; // Current count of how many nodes we have
            // num edges will need to be known for graph planners
            bool foundEnd;
            PointXY _goal;
            float xMin;
            float xMax;
            float yMin;
            float yMax;

            // Random Generators
            std::mt19937 _randGenX;
            std::mt19937 _randGenY;
            std::mt19937 _randGenGoalBias;

            // Floating point distributions for the axes and goal bias
            std::uniform_real_distribution<float> _xDistribution;
            std::uniform_real_distribution<float> _yDistribution;
            std::uniform_real_distribution<float> _goalBiasDistribution;

            // smart pointers to the grid and path validator
            std::shared_ptr<GridManager2D> _Grid;
            std::shared_ptr<IPathValidator2D> _Validator;

            // Config
            const int   _maxIterations;
            const float _maxEdgeLength;
            const float _endBias;
            const float _steerStepSize;
            const float _invalidPenalty;

            PointXY sampleNewNode();

            void steer(PointXY& sampledPoint, PointXY nearestNode, bool& successful);
            
            void rewire();

            float costFunction(const PointXY& sampledPoint, const PointXY& nearestNode);

            int FindNearestNode(PointXY);

    };

}

#endif // RRTSTAR2D_H   