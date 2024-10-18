/*
 * Author: Andrew Campbell
 * Date: 09-17-2024
 */

#ifndef GRIDTYPES2D_H
#define GRIDTYPES2D_H

#include <iostream>
#include <cmath>
#include <vector>

#include "Util/CustomTypes/VectorAndPointTypes.hpp"

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

    struct Obstacle{
        Obstacle( PointXY center, float length, float width, float orientation)
        {
            PointXY points[4];
            // make unit vectors:
            Util::Custom_Types::Vectors::Vector2f e1(std::cos(orientation), std::sin(orientation));
            Util::Custom_Types::Vectors::Vector2f e2(std::cos(orientation + M_PI/2), std::sin(orientation + M_PI/2));

            // TODO Start Here
        };
        Obstacle(PointXY OtherPoints[4]): points(OtherPoints);

        /// @brief Obstacle.points is the four points one can use to define a rectangle in a counterclockwise order
        PointXY points[4];
    };

    class Cell {
        public:
            Cell(Algorithms::TwoD::PointXY p_center, float cellSize)
            {
                _center = p_center;
                _cellSize = cellSize;
                
                _corners = {
                    Algorithms::TwoD::PointXY(p_center.x - (cellSize/2.0f),p_center.y - (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x - (cellSize/2.0f),p_center.y + (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x + (cellSize/2.0f),p_center.y + (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x + (cellSize/2.0f),p_center.y - (cellSize/2.0f)),
                };

                // default if not specified
                _state = State::CLEAR;
                _odds = 0; /// only ranges from [0,1]
            }

            Cell(Algorithms::TwoD::PointXY p_center, float cellSize, State state, float odds)
            {
                _center = p_center;
                _cellSize = cellSize;

                _corners = {
                    Algorithms::TwoD::PointXY(p_center.x - (cellSize/2.0f),p_center.y - (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x - (cellSize/2.0f),p_center.y + (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x + (cellSize/2.0f),p_center.y + (cellSize/2.0f)),
                    Algorithms::TwoD::PointXY(p_center.x + (cellSize/2.0f),p_center.y - (cellSize/2.0f)),
                };

                _state = state;
                _odds = odds;
            }

            Algorithms::TwoD::PointXY getCenter()    
            {
                return _center;
            }

            std::vector<Algorithms::TwoD::PointXY> getCorners()   
            {
                return _corners;
            }

            State getState()    
            {
                return _state;
            }
            void setState(State p_state) 
            {
                _state = p_state;
            }

            void incrementOdds(int increment) 
            {
                _odds += increment;
            }

            float getOdds() 
            {
                return (1 - (1/(1-(std::pow(2, _odds)))));
            }


        private:
            Algorithms::TwoD::PointXY _center;
            std::vector<Algorithms::TwoD::PointXY> _corners;
            State _state;
            float _odds;
            float _cellSize;

    };
}

#endif // GRIDTYPES2D_H