/*
 * Author: Andrew Campbell
 * Date: 09-19-2024
 */

#ifndef RRTSTAR2D_H
#define RRTSTAR2D_H

// base class
#include "IPathPlanner2D.hpp"

namespace Algorithms::TwoD
{
    class RRTStar2D
    {
        public:
            RRTStar2D();
            ~RRTStar2D() = default;

        private:
            struct Tree
            {

            };

            void sampleNewNode();

            void Rewire();

    };

}

#endif // RRTSTAR2D_H   