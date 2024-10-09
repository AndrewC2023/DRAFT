/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#ifndef PLANNINGNODES2D_H
#define PLANNINGNODES2D_H

#include "Util/CustomTypes/VectorAndPointTypes.hpp"

// STL
#include <vector>
#include <deque>

namespace Algorithms::TwoD
{
    // Various datatypes for planning nodes
    /** Sandard RRTStar Node for RRT star path planning
     * @param position The XY postition of the node in real space
     * @param cost the cost to travel from the starting node to this node
     * @param treeIndex the integer index showing where in the tree the node is
     */
    struct RRTStarNode
    {
        PointXY position;
        float cost;
        int treeIndex;
        int parentIndex;
    };

    struct AStarNetworkNode
    {
      PointXY position;
      float costToNode;
      float heuristicCost;
      int networkIndex;
      std::deque<int> networkConnections;
      int parentIndex;
    };

}

#endif // PLANNINGNODES2D_H