/*
 * Author: Andrew Campbell
 * Date: 09-17-2024
 */

#ifndef GRIDTYPES2D_H
#define GRIDTYPES2D_H

#include <iostream>
#include <cmath>
#include <vector>

#include "VectorAndPointTypes.hpp"
#include "Geometry.hpp"

namespace Algorithms::TwoD
{
     struct UpdateRequest    {
        int index;
        float oddsUpdate;
    };

    enum State  {
        CLEAR,
        UNCERTAIN,
        OBSTACLE
    };

    // TODO: Use boost polygons instead the idea of this sturcture is for 
    // satic predefined obstacles (at least for now) so it does not need to be perfectly optimized

    /**
     * Static obstacle class
     * the most basic obstacle type effectively characterised as its 
     */
    class StaticObstacle
    {
        public:
            StaticObstacle(std::vector<PointXY> Corners) : corners(Corners){}; 
            StaticObstacle(PointXY center, float length, float width, float orientation)
            {
                corners = Math::Geometry::generateRectangularOutline(center, length, width, orientation);
            };

            ~StaticObstacle() = default;
            std::vector<PointXY> getCorners(){return corners;}
        private:
            std::vector<PointXY> corners;
    };

    struct UncertainPointXY
    class StaticUncertainObstacle
    {
        // set up such that effectively all paramaters are potentially random, if some arent supposed to be random then construct those parameters' variance as 0 
        public:

        private:
            std::vector<

        // need boundary conditions  
    };

    class DynamicObstacle
    {
        public:
    };

    class DynamicUncertainObstacle
    {
        public:


    };





    // Given how we use this class do we need to make its member values private? would it be faster if we didnt need getter functions? the values do get accessed a lot
    class Cell {
        public:
            Cell(Algorithms::TwoD::PointXY p_center, float cellSize)
            {
                center = p_center;
                cellSize = cellSize;
                
                corners = {
                    Algorithms::TwoD::PointXY(p_center.x - (cellSize/2.0f),p_center.y - (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x - (cellSize/2.0f),p_center.y + (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x + (cellSize/2.0f),p_center.y + (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x + (cellSize/2.0f),p_center.y - (cellSize/2.0f)),
                };

                // default if not specified
                state = State::CLEAR;
                odds = 0; /// only ranges from [0,1]
            }

            Cell(Algorithms::TwoD::PointXY p_center, float cellSize, State state, float odds)
            {
                center = p_center;
                cellSize = cellSize;

                corners = {
                    Algorithms::TwoD::PointXY(p_center.x - (cellSize/2.0f),p_center.y - (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x - (cellSize/2.0f),p_center.y + (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x + (cellSize/2.0f),p_center.y + (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x + (cellSize/2.0f),p_center.y - (cellSize/2.0f)),
                };

                state = state;
                odds = odds;
            }

            Algorithms::TwoD::PointXY getCenter()    
            {
                return center;
            }

            std::vector<Algorithms::TwoD::PointXY> getCorners()   
            {
                return corners;
            }

            State getState()    
            {
                return state;
            }
            void setState(State p_state) 
            {
                state = p_state;
            }

            void incrementOdds(int increment) 
            {
                odds += increment;
            }

            float getOdds() 
            {
                return (1 - (1/(1-(std::pow(2, odds)))));
            }


        private:
            Algorithms::TwoD::PointXY center;
            std::vector<Algorithms::TwoD::PointXY> corners;
            State state;
            float odds;
            float cellSize;

    };
}

#endif // GRIDTYPES2D_H