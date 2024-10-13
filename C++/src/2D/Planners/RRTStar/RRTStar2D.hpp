/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#ifndef RRTSTAR2D_H
#define RRTSTAR2D_H

// base class
#include "IPathPlanner2D.hpp"

// Config
#include "Util/Config/Config.hpp"

// Custom Types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"
#include "PlanningNodes2D.hpp"

//STL
#include <vector>
#include <memory>

namespace Algorithms::TwoD
{
    class RRTStar2D : IPathPlanner2D
    {
        public:
            RRTStar2D(Configuration::Config&, 
                      std::shared_ptr<GridManager>,
                      std::shared_ptr<IPathValidator>);

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

            // smart pointers to the grid and path validator
            std::shared_ptr<GridManager> _Grid;
            std::shared_ptr<IPathValidator> _Validator;

            // Config
            const int _maxIterations;
            const float _maxEdgeLength;
            const float _endBias;

            // other constants
            const float minX;
            const float maxX;
            const float minY;
            const float maxY;

            PointXY sampleNewNode();

            void steer(PointXY&, PointXY, bool&);
            
            void Rewire();

            float costFunction();

    };

}

#endif // RRTSTAR2D_H   