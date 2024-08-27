/*
 * Author: Andrew Campbell
 * Date: 08-27-2024
 */

#ifndef IGRIDMANAGER2D_H
#define IGRIDMANAGER2D_H

#include "Util/CustomTypes/VectorAndPointTypes.hpp"

// STL
#include <vector>

namespace Algorithms::TwoD
{

    class IGridManager2D
    {
        public:

        virtual bool isOccupied();

        virtual void updateGrid(std::vector<PointXY>);
        virtual void updateGrid(std::vector<IndexXY>);

    };
}

#endif // IGRIDMANAGER2D_H