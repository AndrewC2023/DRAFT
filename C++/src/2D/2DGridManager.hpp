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

    class GridManager   {
        public:
        GridManager(float p_maxX, float p_maxY, float p_cellSize);
        ~GridManager();

        // returns vector representation of grid
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

        void addObstacles(float x, float y);
        void addObstacles(PointXY point);

        void pushUpdatesToGrid();

        
        private:
        std::vector<Cell> _grid;
        
        std::deque<UpdateRequest> _updates;
        std::mutex _cellUpdateMutex;
        
        std::atomic<bool> _cellUpdateThreadRunning;
        const int _cellUpdateThreadPeriod_ms = 5;
        std::thread _cellUpdateThread;

        float cellSize;
        float xSize;
        float ySize;

        bool initialized = false;

        float positveThreshold = 0.95;
        float clearThreshold = 0.05;

        void createEmptyGrid(float p_maxX, float p_maxY, float p_cellSize);

    };
} 
