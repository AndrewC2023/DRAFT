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
#include "Util/CustomTypes/VectorAndPointTypes.hpp"
#include "GridTypes3D.hpp"

namespace Algorithms::ThreeD {

    class GridManager   {
        public:
        GridManager(float p_maxX, float p_maxY, float p_maxZ, float p_cellSize);
        ~GridManager();

        // returns vector representation of grid
        std::vector<Cell> getCells();

        // returns cell at x, y, z
        Cell getCell(float x, float y, float z);
        // returns cell at center point
        Cell getCell(PointXYZ point);

        // gets neighbors of cells given the specific cell and a depth
        std::vector<Cell> getNeighbors(Cell cell, int depth);
        // gets neighbors of cells given a point in space -> point will be normalized to the cell that contains it
        std::vector<Cell> getNeighbors(float x, float y, float z, int depth);
        // gets neighbors of cells given a PointXYZ representation of a point in space -> will be normailized to containing cell
        std::vector<Cell> getNeighbors(PointXYZ point);

        void addObstacles(float x, float y, float z);
        void addObstacles(PointXYZ point);

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
        float zSize;

        bool initialized = false;

        float positveThreshold = 0.95;
        float clearThreshold = 0.05;

        void createEmptyGrid(float p_maxX, float p_maxY, float p_maxZ, float p_cellSize);

        

    };
}