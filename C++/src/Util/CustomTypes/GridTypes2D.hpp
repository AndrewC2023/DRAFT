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

    // TODO: Use boost polygohns instead the idea of this sturcture is for 
    // satic predefined obstacles (at least for now) so it does not need to be perfectly optimized

    struct Obstacle{
        Obstacle( PointXY center, float length, float width, float orientation)
        {
            // make unit vectors:
            Util::Custom_Types::Vectors::Vector2f e1(std::cos(orientation), std::sin(orientation));
            Util::Custom_Types::Vectors::Vector2f e2(std::cos(orientation + M_PI/2), std::sin(orientation + M_PI/2));

            float hL = length/2;
            float hW = width/2;

            points[1] = center + e1*hL - e2*hW;
            points[2] = center + e1*hL + e2*hW;
            points[3] = center - e1*hL + e2*hW;
            points[4] = center - e1*hL - e2*hW;
        };
        Obstacle(PointXY OtherPoints[4])
        {
            for(int i = 0; i < 4; i++)
            {
                points[i] = OtherPoints[i];
            }
        };

        /// @brief Obstacle.points is the four points one can use to define a rectangle in a counterclockwise order
        PointXY points[4];
        const State state = State::OBSTACLE;
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