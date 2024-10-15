//
// Created by Andrew C on 04/19/24
//

#include "2DGridManager.hpp"
//
namespace Algorithms::TwoD {
    // TODO: implement pre calculation of number of cells -> mild performance saving
    GridManager::GridManager(float p_minX, float p_maxX,
                             float p_minY, float p_maxY,
                             float p_cellSize):
                            _cellUpdateThreadRunning(true),
                            _xSize(p_maxX - p_minX),
                            _ySize(p_maxY - p_minY),
                            _cellSize(p_cellSize),
                            _xMin(p_minX),
                            _xMax(p_maxX),
                            _yMin(p_minY),
                            _yMax(p_maxY)
    {
        int _numCellsX = static_cast<int>(_xSize / _cellSize);
        int _numCellsY = static_cast<int>(_ySize / _cellSize);
        createEmptyGrid();
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
    void GridManager::createEmptyGrid() {
        int numX = static_cast<int>(_xSize / _cellSize);
        int numY = static_cast<int>(_ySize / _cellSize);

        for (int i = 0; i < numX; i++)  {
            for (int j = 0; j < numY; j++)  {
                
                float centerX = (i + 0.5) * _cellSize + _xMin;
                float centerY = (j + 0.5) * _cellSize + _yMin;

                _grid.emplace_back(Cell(PointXY(centerX, centerY), _cellSize));
            }
        }
        initialized = true;
    }

    // returns the cell that contains point at x, y, z
    Cell GridManager::getCell(float x, float y) {

        int cellIndexX = static_cast<int>(std::floor((x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::floor((y - _yMin) / _cellSize));

        if (cellIndexX < 0 || cellIndexX >= _numCellsX || cellIndexY < 0 || cellIndexY >= _numCellsY)    {
            // TODO: implement actuall error handling system -> could log and return -1, -1, -1 just null for now
            std::cout << "WARNING: attempted access of grid outside of defined domain\n";
            // return a meaningless cell that is listed as an obstacle
            return Cell(PointXY(0.0f,0.0f), _cellSize, State::OBSTACLE,1.0f);
        }

        int calcIndex = cellIndexX + _numCellsX * (cellIndexY + _numCellsY);

        return _grid[calcIndex];
    }

    // returns the cell that contains PointXYZ point
    Cell GridManager::getCell(PointXY point) 
    {
        int cellIndexX = static_cast<int>(std::floor((point.x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::floor((point.y - _yMin) / _cellSize));

        if (cellIndexX < 0 || cellIndexX >= _numCellsX || cellIndexY < 0 || cellIndexY >= _numCellsY)    {
            // TODO: implement actuall error handling system -> could log and return -1, -1, -1 just null for now
            std::cout << "WARNING: attempted access of grid outside of defined domain\n";
            // return a meaningless cell that is listed as an obstacle
            return Cell(PointXY(0.0f,0.0f), _cellSize, State::OBSTACLE,1.0f);
        }

        int calcIndex = cellIndexX + _numCellsX * (cellIndexY);

        return _grid[calcIndex];
    }

    // TODO: implement caching. depends on what hardware this runs on bc if we dont care abt space we could just hold a massive lookup table tbh
    std::vector<Cell> GridManager::getNeighbors(float x, float y, int depth)   
    {
        std::vector<Cell> neighbors;

        int cellIndexX = static_cast<int>(std::floor((x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::floor((y - _yMin) / _cellSize));
        // TODO: there is a deterministic way to implement this given we know how the list is structured
        // TODO: error handling?
        int minX = std::max(0, cellIndexX - depth);
        int maxX = std::min(_numCellsX - 1, cellIndexX + depth);
        int minY = std::max(0, cellIndexY - depth);
        int maxY = std::min(_numCellsY - 1, cellIndexY + depth);

        for (int i = minX; i < maxX; i++)   {
            for (int j = minY; j < maxY; j++)   {
                int neighborIndex = x + _numCellsX * (y);
                neighbors.push_back(_grid[neighborIndex]);
            }
        }

        return neighbors;
    }

    void GridManager::addKnownObstacle(float x, float y)  
    {

        int cellIndexX = static_cast<int>(std::floor((x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::floor((y - _yMin) / _cellSize));

        bool invalid = cellIndexX < 0 || cellIndexX >= _numCellsX || cellIndexY < 0 || cellIndexY >= _numCellsY;

        if (!invalid)   {
            std::scoped_lock lock(_cellUpdateMutex);
            
            int calcIndex = cellIndexX + _numCellsX * (cellIndexY);

            _updates.push_back(UpdateRequest{calcIndex, 1});
        }
    }

    void GridManager::addKnownObstacle(PointXY point)   {
        int _numCellsX = static_cast<int>(_xSize / _cellSize);
        int _numCellsY = static_cast<int>(_ySize / _cellSize);

        int cellIndexX = static_cast<int>(std::floor((point.x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::floor((point.y - _yMin) / _cellSize));

        bool invalid = cellIndexX < 0 || cellIndexX >= _numCellsX || cellIndexY < 0 || cellIndexY >= _numCellsY;

        if (!invalid)   {
            std::scoped_lock lock(_cellUpdateMutex);
            
            int calcIndex = cellIndexX + _numCellsX * (cellIndexY);

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
