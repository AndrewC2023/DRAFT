/*
 *Author: Andrew Campbell
 * Date: 12-05-2024
 */

#ifndef TWODHYBRIDGRIDMANAGER_H
#define TWODHYBRIDGRIDMANAGER_H

// std
#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>

// custom types
#include "Util/CustomTypes/GridTypes2D.hpp"

namespace Algorithms::TwoD
{
    class HybridGridManager2D   {
        public:
        /** GridManager 2D
         *  @param p_minX the minimum X value in true space
         *  @param p_maxX the maximum X value in true space
         *  @param p_minY the minimum Y value in true space
         *  @param p_maxY the maximum Y value in true space
         *  @param p_cellsize the side length of a square cell in true space units (meters
         */
        HybridGridManager2D(float p_minX, float p_maxX,
                    float p_minY, float p_maxY,
                    float p_cellSize);
        ~HybridGridManager2D();

        // returns vector representation of grid (same type as _grid)
        std::vector<Cell> getCells(float);

        // returns cell at x, y, z
        Cell getCell(float x, float y);
        // returns cell at center point
        Cell getCell(PointXY point);

        // gets neighbors of cells given the specific cell and a depth
        std::vector<Cell> getNeighbors(Cell cell, int depth);
        // gets neighbors of cells given a point in space -> point will be normalized to the cell that contains it
        std::vector<Cell> getNeighbors(float x, float y, int depth);
        // gets neighbors of cells given a PointXYZ representation of a point in space -> will be normailized to containing cell
        std::vector<Cell> getNeighbors(PointXY point);

        /** Add Known Obstacles method
         *  @param x,y the x and y position that we want to set as occupied
         *  will find the cell that contains this point and set it as occupied
         */
        void addKnownObstacle(float x, float y);
        /** Add Known Obstacles method
         *  @param point the point that we want to set as occupied
         *  will find the cell that contains this point and set it as occupied
         */
        void addKnownObstacle(PointXY point);
        /** Add Known Obstacles method
         *  @param obstacle the x and y position that we want to set as occupied
         *  will find the cell that contains this point and set it as occupied
         */
        void addKnownObstacle(I2DObstacle obstacle); // TODO: change to boost polygon and intersect!

        /** get the nearest obstacle center
         *  @param checkPoint the point that we want to see what the nearest obstacle is to
         *  @return the center of the nearest obstacle
         */
        PointXY getNearestObstacleCenter(PointXY checkPoint);

        /** get the nearest obstacle center
         *  @param checkPoint the point that we want to see what the nearest obstacle is to
         *  @return the intersection point of the stright line projected from the check point to the center of the nearest obstacle
         */
        float getDistanceToNearestObstacle(PointXY checkPoin); // may want to allow for the return of the intersection point?

        void pushUpdatesToGrid();

        /** Helper to get the grid domain values
         * @param xMin,xMax,yMin,yMax modified by reference
         */
        void getGridDomain(float& xMin, float& xMax, float& yMin, float& yMax);

        private:
        // The grid as a list of Grid Cells
        std::vector<Cell> _grid;

        // The grid will also cointain a list of known obstacles, This allows for the opportunity to potential speed up path validation
        std::vector<I2DObstacle> _obstacleList;


        // The updates to be made to the grid
        std::deque<UpdateRequest> _updates;
        std::mutex _cellUpdateMutex;
        
        std::atomic<bool> _cellUpdateThreadRunning;
        const int _cellUpdateThreadPeriod_ms = 5;
        std::thread _cellUpdateThread;

        /// grid domain descriptors
        float _cellSize;
        float _xSize;
        float _ySize;
        float _xMin;
        float _xMax;
        float _yMin;
        float _yMax;
        int _numCellsX;
        int _numCellsY;

        bool initialized = false;

        float positveThreshold = 0.95;
        float clearThreshold = 0.05;

        void createEmptyGrid();

    };
}

#endif // TWODHYBRIDGRIDMANAGER_H