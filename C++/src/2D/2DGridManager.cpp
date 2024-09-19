//
// Created by Andrew C on 04/19/24
//

#include "2DGridManager.hpp"

namespace Algorithms::TwoD {
    // TODO: implement pre calculation of number of cells -> mild performance saving
    GridManager::GridManager(float p_maxX, float p_maxY, float p_cellSize):
                            _cellUpdateThreadRunning(true) {
        xSize = p_maxX;
        ySize = p_maxY;
        cellSize = p_cellSize;

        createEmptyGrid(p_maxX, p_maxY, p_cellSize);

        _cellUpdateThread = std::thread([this](){ pushUpdatesToGrid(); });
    }

    GridManager::~GridManager() {
        _cellUpdateThreadRunning = false;
        if(_cellUpdateThread.joinable())
            _cellUpdateThread.join();
    }

    std::vector<Cell> GridManager::getCells()   {
        if (initialized)    {
            return _grid;
        }
        else    {
            // TODO: implement error handling here -> actual logging for now just return grid
            return _grid;
        }
    }

    //TODO: implement multithreaded approach to make grid update
    // e.g updater thread that we push updates to so the thread that holds the grid doesnt risk getting throttled under large data load
    void GridManager::createEmptyGrid(float p_maxX, float p_maxY, float p_cellSize) {
        int numX = static_cast<int>(p_maxX / p_cellSize);
        int numY = static_cast<int>(p_maxY / p_cellSize);

        for (int i = 0; i < numX; i++)  {
            for (int j = 0; j < numY; j++)  {
                
                float centerX = (i + 0.5) * p_cellSize;
                float centerY = (j + 0.5) * p_cellSize;

                _grid.emplace_back(Cell(PointXY(centerX, centerY), p_cellSize));
            }
        }
        initialized = true;
    }

    // returns the cell that contains point at x, y, z
    Cell GridManager::getCell(float x, float y) {
        int numCellsX = static_cast<int>(xSize / cellSize);
        int numCellsY = static_cast<int>(ySize / cellSize);

        int cellIndexX = static_cast<int>(std::floor(x / cellSize));
        int cellIndexY = static_cast<int>(std::floor(y / cellSize));

        if (cellIndexX < 0 || cellIndexX >= numCellsX || cellIndexY < 0 || cellIndexY >= numCellsY)    {
            // TODO: implement actuall error handling system -> could log and return -1, -1, -1 just null for now
            return _grid[0];
        }

        int calcIndex = cellIndexX + numCellsX * (cellIndexY + numCellsY);

        return _grid[calcIndex];
    }

    // returns the cell that contains PointXYZ point
    Cell GridManager::getCell(PointXY point) {
        int numCellsX = static_cast<int>(xSize / cellSize);
        int numCellsY = static_cast<int>(ySize / cellSize);

        int cellIndexX = static_cast<int>(std::floor(point.x / cellSize));
        int cellIndexY = static_cast<int>(std::floor(point.y / cellSize));

        if (cellIndexX < 0 || cellIndexX >= numCellsX || cellIndexY < 0 || cellIndexY >= numCellsY)    {
            // TODO: implement actuall error handling system -> could log and return -1, -1, -1 just null for now
            return _grid[0];
        }

        int calcIndex = cellIndexX + numCellsX * (cellIndexY);

        return _grid[calcIndex];
    }

    // TODO: implement caching. depends on what hardware this runs on bc if we dont care abt space we could just hold a massive lookup table tbh
    std::vector<Cell> GridManager::getNeighbors(float x, float y, int depth)   {
        std::vector<Cell> neighbors;
        
        int numCellsX = static_cast<int>(xSize / cellSize);
        int numCellsY = static_cast<int>(ySize / cellSize);

        int cellIndexX = static_cast<int>(std::floor(x / cellSize));
        int cellIndexY = static_cast<int>(std::floor(y / cellSize));

        int minX = std::max(0, cellIndexX - depth);
        int maxX = std::min(numCellsX - 1, cellIndexX + depth);
        int minY = std::max(0, cellIndexY - depth);
        int maxY = std::min(numCellsY - 1, cellIndexY + depth);

        for (int i = minX; i < maxX; i++)   {
            for (int j = minY; j < maxY; j++)   {
                int neighborIndex = x + numCellsX * (y);
                neighbors.push_back(_grid[neighborIndex]);
            }
        }

        return neighbors;
    }

    void GridManager::addObstacles(float x, float y)   {
        int numCellsX = static_cast<int>(xSize / cellSize);
        int numCellsY = static_cast<int>(ySize / cellSize);

        int cellIndexX = static_cast<int>(std::floor(x / cellSize));
        int cellIndexY = static_cast<int>(std::floor(y / cellSize));

        bool invalid = cellIndexX < 0 || cellIndexX >= numCellsX || cellIndexY < 0 || cellIndexY >= numCellsY;

        if (!invalid)   {
            std::scoped_lock lock(_cellUpdateMutex);
            
            int calcIndex = cellIndexX + numCellsX * (cellIndexY);

            _updates.push_back(UpdateRequest{calcIndex, 1});
        }
    }

    void GridManager::addObstacles(PointXY point)   {
        int numCellsX = static_cast<int>(xSize / cellSize);
        int numCellsY = static_cast<int>(ySize / cellSize);

        int cellIndexX = static_cast<int>(std::floor(point.x / cellSize));
        int cellIndexY = static_cast<int>(std::floor(point.y / cellSize));

        bool invalid = cellIndexX < 0 || cellIndexX >= numCellsX || cellIndexY < 0 || cellIndexY >= numCellsY;

        if (!invalid)   {
            std::scoped_lock lock(_cellUpdateMutex);
            
            int calcIndex = cellIndexX + numCellsX * (cellIndexY);

            _updates.push_back(UpdateRequest{calcIndex, 1});
        }
    }

    void GridManager::pushUpdatesToGrid()   {
        auto goalTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(_cellUpdateThreadPeriod_ms);

        while (_cellUpdateThreadRunning)    {
            std::size_t updateNum = 0;

            {
                std::scoped_lock updateCountLock(_cellUpdateMutex);
                updateNum = _updates.size();
            }

            if (updateNum > 0)  {
                std::scoped_lock updatesLock(_cellUpdateMutex);

                for (auto& request : _updates)   {
                    _grid[request.index].incrementOdds(request.oddsUpdate);
                    int oddsVal = _grid[request.index].getOdds();
                    if (oddsVal >= positveThreshold) {
                        _grid[request.index].setState(OBSTACLE);
                    }
                    else if(oddsVal <= clearThreshold)  {
                        _grid[request.index].setState(CLEAR);
                    }
                    else    {
                        _grid[request.index].setState(UNCERTAIN);
                    }
                }
                _updates.clear();
            }
            else    {
                std::this_thread::sleep_for(std::chrono::milliseconds(_cellUpdateThreadPeriod_ms));
            }

        }

    }

}
