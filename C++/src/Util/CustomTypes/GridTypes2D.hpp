/*
 * Author: Andrew Campbell
 * Date: 09-17-2024
 */

#ifndef GRIDTYPES2D_H
#define GRIDTYPES2D_H

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>

#include "VectorAndPointTypes.hpp"
#include "Util/Math/Geometry.hpp"

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


    // Region Obstacle Classes

    // Uncertain obstacles currently only support independed uncertain variables
        // TODO: add an obstacle factory
        // Base obstacle class wrapper
        class I2DObstacle
        {
            public:
                virtual ~I2DObstacle() = default; // Virtual destructor for polymorphism

                virtual const std::vector<PointXY>& getCorners() = 0;
        };

        /**
         * Static obstacle class
         * the most basic obstacle type effectively characterised as its 
         * @param corners this gives the cornor locations of the polygon describing the obstace
         */
        class StaticObstacle : public I2DObstacle
        {
            public:
                StaticObstacle(std::vector<PointXY> Corners) : corners(Corners){}; 
                StaticObstacle(PointXY center, float length, float width, float orientation)
                {
                    corners = Math::Geometry::generateRectangularOutline(center, length, width, orientation);
                };

                ~StaticObstacle() = default;

                const std::vector<PointXY>& getCorners() override {return corners;}

            private:
                std::vector<PointXY> corners;
        };

        class StaticUncertainObstacle : public I2DObstacle
        {
            // set up such that effectively all paramaters are potentially random, if some arent supposed to be random then construct those parameters' variance as 0 
            public:
                StaticUncertainObstacle(){};

                const std::vector<PointXY>& getCorners() override {return corners;}
            private:
                std::vector<PointXY> corners; // treated as a mean effectively
                std::vector<std::pair<float, float>> CornerVariance; // TODO: this needs to eventually support any PDF and not just normal
                PointXY centroid;

                
        };

        class DynamicObstacle : public I2DObstacle // TODO: Finish this class
        {
            public:
                // This class needs dimensions and initial conditions
                std::vector<PointXY> propagateInTime(float finalTime, float dt);

                const std::vector<PointXY>& getCorners() override {return corners;}
            private:
                std::vector<PointXY> corners; // these points are given and one should consider the centroid the CM unless specified
                float t_0;
                PointXY CM_initialPosition;
                // Dynamics
                // TODO: wtf do i do here
                
                // IDynamics dynamics;
                
                // Boundary Conditions
                float initialTime;
                float initialCMPosition;
            // need boundary conditions  
        };

        class DynamicUncertainObstacle : public I2DObstacle // TODO: make more than the inputs possibly uncertain
        {
            public:
                // This class needs dimensions and initial conditions
                std::vector<PointXY> propagateInTime(float finalTime, float dt);

                const std::vector<PointXY>& getCorners() override {return corners;}
            private:
                std::vector<PointXY> corners; // these points are given and one should consider the centroid the CM unless specified
                float t_0;
                PointXY CM_initialPosition;
                // Dynamics
                // TODO: wtf do i do here
                
                // IDynamics dynamics;
                
                // Boundary Conditions
                float initialTime;
                float initialCMPosition;
            // need boundary conditions  
        };


    // End Region



    // Given how we use this class do we need to make its member values private? would it be faster if we didnt need getter functions? the values do get accessed a lot
    class Cell {
        public:
            Cell(Algorithms::TwoD::PointXY p_center, Algorithms::TwoD::IndexXY Index, float cellSize)
            {
                index = Index;
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

            Cell(Algorithms::TwoD::PointXY p_center, Algorithms::TwoD::IndexXY Index, float cellSize, State state, float odds)
            {
                index = Index;
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

            Algorithms::TwoD::PointXY getCenter() const
            {
                return center;
            }

            const std::vector<Algorithms::TwoD::PointXY> getCorners() const   
            {
                return corners;
            }

            Algorithms::TwoD::IndexXY getIndex() const
            {
                return index;
            }

            State getState() const
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

            const float getOdds() const 
            {
                return (1 - (1/(1-(std::pow(2, odds)))));
            }


        private:
            Algorithms::TwoD::PointXY center;
            std::vector<Algorithms::TwoD::PointXY> corners;
            Algorithms::TwoD::IndexXY index;
            State state;
            float odds;
            float cellSize;


    };
}

#endif // GRIDTYPES2D_H