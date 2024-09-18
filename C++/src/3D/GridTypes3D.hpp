#ifndef GRIDTYPES3D_H
#define GRIDTYPES3D_H

#include <iostream>
#include <cmath>
#include <vector>

#include "Util/CustomTypes/VectorAndPointTypes.hpp"

namespace Algorithms::ThreeD {

    struct UpdateRequest    {
        int index;
        float oddsUpdate;
    };

    enum State  {
        CLEAR,
        UNCERTAIN,
        OBSTACLE
    };

    class Cell {
        public:
            Cell(Algorithms::ThreeD::PointXYZ p_center, float cellSize)
            {
                _center = p_center;
                _cellSize = cellSize;
                
                _corners = {
                    Algorithms::ThreeD::PointXYZ(p_center.x - (cellSize/2.0f),p_center.y - (cellSize/2.0f),p_center.z - (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x - (cellSize/2.0f),p_center.y + (cellSize/2.0f),p_center.z - (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x - (cellSize/2.0f),p_center.y + (cellSize/2.0f),p_center.z + (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x - (cellSize/2.0f),p_center.y - (cellSize/2.0f),p_center.z + (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x + (cellSize/2.0f),p_center.y - (cellSize/2.0f),p_center.z - (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x + (cellSize/2.0f),p_center.y + (cellSize/2.0f),p_center.z - (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x + (cellSize/2.0f),p_center.y + (cellSize/2.0f),p_center.z + (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x + (cellSize/2.0f),p_center.y - (cellSize/2.0f),p_center.z + (cellSize/2.0f)),
                };

                // default if not specified
                _state = State::CLEAR;
                _odds = 0;
            }

            Cell(Algorithms::ThreeD::PointXYZ p_center, float cellSize, State state, float odds)
            {
                _center = p_center;
                _cellSize = cellSize;

                _corners = {
                    Algorithms::ThreeD::PointXYZ(p_center.x - (cellSize/2.0f),p_center.y - (cellSize/2.0f),p_center.z - (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x - (cellSize/2.0f),p_center.y + (cellSize/2.0f),p_center.z - (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x - (cellSize/2.0f),p_center.y + (cellSize/2.0f),p_center.z + (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x - (cellSize/2.0f),p_center.y - (cellSize/2.0f),p_center.z + (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x + (cellSize/2.0f),p_center.y - (cellSize/2.0f),p_center.z - (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x + (cellSize/2.0f),p_center.y + (cellSize/2.0f),p_center.z - (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x + (cellSize/2.0f),p_center.y + (cellSize/2.0f),p_center.z + (cellSize/2.0f)),
                    Algorithms::ThreeD::PointXYZ(p_center.x + (cellSize/2.0f),p_center.y - (cellSize/2.0f),p_center.z + (cellSize/2.0f)),
                };

                _state = state;
                _odds = odds;
            }

            Algorithms::ThreeD::PointXYZ getCenter()    
            {
                return _center;
            }

            std::vector<Algorithms::ThreeD::PointXYZ> getCorners()   
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
            Algorithms::ThreeD::PointXYZ _center;
            std::vector<Algorithms::ThreeD::PointXYZ> _corners;
            State _state;
            float _odds;
            float _cellSize;

    };
}

#endif // GRIDTYPES3D_H