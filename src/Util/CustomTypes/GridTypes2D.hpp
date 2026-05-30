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
#include <random>

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

        class DynamicObstacle : public I2DObstacle // TODO: Finish this class and fix it, so so so much to fix here
        {
            public:
            DynamicObstacle(std::vector<PointXY> Corners,
                            StateXYT BoundaryConditionState, 
                            float boundaryConditionInitialTime,
                            float forwardVelocity,
                            float TurnRate)
                            : corners(Corners),
                              Pose_0(BoundaryConditionState),
                              t_0(boundaryConditionInitialTime),
                              velocity(forwardVelocity),
                              turnRate(TurnRate)
            {}
            const std::vector<PointXY>& getCorners() override { return corners; }

            std::vector<StateXYT> propagateInTime(float finalTime, float dt, int subResolution)
                {
                    std::vector<StateXYT> trajectory;
                    float t = t_0;
                    StateXYT position = Pose_0;
                    trajectory.push_back(Pose_0);

                    int subcount = 1;
                    
                    while (t < finalTime)
                    {
                        
                        // Update position based on differential drive model
                        position.x += velocity * std::cos(position.z) * dt/subResolution;
                        position.y += velocity * std::sin(position.z) * dt/subResolution;
                        position.z += turnRate * dt/subResolution;
                        
                        if(subcount == subResolution)
                        {
                            trajectory.push_back(position);
                            t += dt;
                            subcount = 1;
                        }else{
                            subcount++;
                        }
                    }

                    return trajectory;
                }

            void setTurnRate(float rate) { turnRate = rate; }

            private:
            std::vector<PointXY> corners; // these points are given and one should consider the centroid the CM unless specified
            float t_0;
            StateXYT Pose_0;
            float velocity; // constant forward velocity
            float turnRate; // turn rate as input
        };


        // TODO: so many, a multi week to moth project to get these to the level of functionality they need to be
        // need any typre of dynamics, any amout of uncertainty, and all methods withing to manage
        class DynamicUncertainObstacle : public I2DObstacle // TODO: make more than the inputs possibly uncertain
        {
            public:
                /**
                 * @brief 
                 * 
                 * @param finalTime the final time to generate the possible futures to, initial time will be a member of the class
                 * @param dt time step
                 * @param numSamples the total number of samples of futures to generate 
                 * 
                 * @note the grid that calls this method must keep track of the time, time,step, anf initial time, 
                 * for now it needs to keep all of these things synchronized utiol the grid id more advanced
                 * 
                 * @return std::vector<std::vector<StateXYT>> this is the vector of the possible futures 
                 * where the posible future is represented as a vectore of the same size as total time/dt so that the
                 * second vector represents all time steps of a single sample in the set of samples
                 */
                
                DynamicUncertainObstacle(std::vector<PointXY> Corners, 
                                         StateXYT boundaryConditionState, 
                                         float boundaryConditionInitialTime,
                                         float boundaryConditionVelocityMean, 
                                         float boundaryConditionVelocityVariance, 
                                         float boundaryConditionTurnRateMean,
                                         float boundaryConditionTurnRateVariance)
                                         : corners(Corners), 
                                           initialCMstate(boundaryConditionState), 
                                           initialTime(boundaryConditionInitialTime), 
                                           velocityMean(boundaryConditionVelocityMean),
                                           velocityVariance(boundaryConditionVelocityVariance), 
                                           turnRateMean(boundaryConditionTurnRateMean),
                                           turnRateVariance(boundaryConditionTurnRateVariance),
                                           randomGenerator1({std::random_device{}()}),
                                           randomGenerator2({std::random_device{}()})
                {
                    normDistTurnRate = std::normal_distribution(boundaryConditionTurnRateMean , boundaryConditionTurnRateVariance);
                    normDistVelocity = std::normal_distribution(boundaryConditionVelocityMean , boundaryConditionVelocityVariance);
                }

                std::vector<std::vector<StateXYT>> GeneratePossibleFutures(float finalTime, float dt, int subResolution, int numSamples)
                {
                    std::vector<std::vector<StateXYT>> possibleFutures;
                    for(int i = 0; i < numSamples; i++)
                    {
                        
                        possibleFutures.push_back(propagateInTime(finalTime, dt, subResolution));
                    }
                    return possibleFutures;
                }

                // This class needs dimensions and initial conditions
                std::vector<StateXYT> propagateInTime(float finalTime, float dt, int subResolution)
                {
                    std::vector<StateXYT> trajectory;
                    float t = initialTime;
                    StateXYT position = initialCMstate;
                    trajectory.push_back(initialCMstate);

                    int subcount = 1;
                    float turnRate = normDistTurnRate(randomGenerator1);
                    float velocity = normDistVelocity(randomGenerator2);

                    while (t < finalTime)
                    {
                        
                        // Update position based on differential drive model
                        position.x += velocity * std::cos(position.z) * dt/subResolution;
                        position.y += velocity * std::sin(position.z) * dt/subResolution;
                        position.z += turnRate * dt/subResolution;
                        
                        if(subcount == subResolution)
                        {
                            trajectory.push_back(position);
                            t += dt;
                            turnRate = normDistTurnRate(randomGenerator1);
                            velocity = normDistVelocity(randomGenerator2);
                            subcount = 1;
                        }else{
                            subcount++;
                        }
                    }

                    return trajectory;
                }

                const std::vector<PointXY>& getCorners() override {return corners;}

            private:
                std::vector<PointXY> corners; // these points are given and one should consider the centroid the CM unless specified
                float initialTime;
                StateXYT initialCMstate;    
                float velocityMean; // constant forward velocity
                float velocityVariance; // constant forward velocity
                float turnRateMean; // turn rate as input
                float turnRateVariance; // turn rate as input

                // random generators:
                std::mt19937 randomGenerator1;
                std::mt19937 randomGenerator2;
                std::normal_distribution<float> normDistTurnRate;
                std::normal_distribution<float> normDistVelocity;
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

            void incrementOdds(float increment) 
            {
                odds += increment;
                if (odds > 1.0f)
                {
                    odds = 1.0f;
                }else if(odds < 0.0f)
                {
                    odds = 0.0f;
                }
            }

            const float getOdds() const 
            {
                // TODO: figure this out more
                return odds;
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