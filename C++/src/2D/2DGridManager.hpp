/*
 * Author: Andrew Campbell
 * Date: 04-19-2024
 */

// std
#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

// custom types
#include "GridTypes2D.hpp"

namespace Algorithms::TwoD {

    /* TODOs:
     *     Allow this grid to be resized after it 
     * has been constructed without losing the current 
     * obstacle set that remains in the domain
     *     Add obstalce retangle and body type 
     * and add obstacle methods for that type
     */
    class GridManager   {
        public:
        /** GridManager 2D
         *  @param p_minX the minimum X value in true space
         *  @param p_maxX the maximum X value in true space
         *  @param p_minY the minimum Y value in true space
         *  @param p_maxY the maximum Y value in true space
         *  @param p_cellsize the side length of a square cell in true space units (meters
         */
        GridManager(float p_minX, float p_maxX,
                    float p_minY, float p_maxY,
                    float p_cellSize);
        ~GridManager();

        // returns vector representation of grid (same type as _grid)
        std::vector<Cell> getCells();

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

        /** add Obstacles method
         * 
         */
        void addKnownObstacle(float x, float y);
        void addKnownObstacle(PointXY point);
        void addKnownObstacle(); // TODO: obstacle type

        void pushUpdatesToGrid();

        
        private:
        // The grid as a list of Grid Cells
        std::vector<Cell> _grid;
        
        // The updates to be made to the grid
        std::deque<UpdateRequest> _updates;
        std::mutex _cellUpdateMutex;
        
        std::atomic<bool> _cellUpdateThreadRunning;
        const int _cellUpdateThreadPeriod_ms = 5;
        std::thread _cellUpdateThread;

        // grid domain descriptors
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
