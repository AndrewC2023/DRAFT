/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#ifndef RRTSTAR2D_H
#define RRTSTAR2D_H

// base class
#include "IPathPlanner2D.hpp"

// Dependent Classes
#include "2DGridManager.hpp"
#include "IPathValidator2D.hpp"

// Config
#include "Util/Config/Config.hpp"

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"
#include "PlanningNodes2D.hpp"

//STL
#include <vector>
#include <memory>
#include <cmath>
#include <random>

namespace Algorithms::TwoD
{
    class RRTStar2D : IPathPlanner2D
    {
        public:
            RRTStar2D(Configuration::Config&, 
                      std::shared_ptr<GridManager>,
                      std::shared_ptr<IPathValidator2D>);

            ~RRTStar2D() = default;

            std::deque<PointXY> PlanPath(PointXY start, PointXY goal) override;
            float getPathCost() override;

        private:
            // The tree of nodes
            std::vector<RRTStarNode> Tree;
            float pathCost;

            int numNodes; // Current count of how many nodes we have
            PointXY start; // Start point
            PointXY goal; // goal point     
            bool foundEnd;
            float xMin;
            float xMax;
            float yMin;
            float yMax;

            // make our random devices
            std::mt19937 randGen1 {std::random_device{}()};
            std::mt19937 randGen2 {std::random_device{}()};
            std::mt19937 randGen3 {std::random_device{}()};

            // Create floating point distributions for the x and y axes
            std::uniform_real_distribution<float> _xDistribution;
            std::uniform_real_distribution<float> _yDistribution;
            std::uniform_real_distribution<float> _goalBiasDistribution;

            // smart pointers to the grid and path validator
            std::shared_ptr<GridManager> Grid;
            std::shared_ptr<IPathValidator2D> Validator;

            // Config
            const int _maxIterations;
            const float _maxEdgeLength;
            const float _endBias;

            PointXY sampleNewNode();

            void steer(PointXY&, PointXY, bool&);
            
            void Rewire();

            float costFunction();

            void setDomain();

            int FindNearestNode(PointXY);
            
    };

}

#endif // RRTSTAR2D_H   