//
// Created by Andrew C on 04/19/24
//


/*
   TODO: boost library should just be used to generalize most obstacles
   this however may not be best for dynamic obstacles or anything tha must check intersections
   more than just on initialization or on adding an obstacle, for validation and propagation of 
   dynamic obstacles it may be necessary to write a much faster custom method rather than relying
   on the robust built in methods of boost due to the potential overhead
*/
#include "2DGridManager.hpp"

namespace Algorithms::TwoD 
{
    // TODO: implement pre calculation of number of cells -> mild performance saving
    GridManager2D::GridManager2D(float p_minX, float p_maxX,
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
        _numCellsX = static_cast<int>(_xSize / _cellSize);
        _numCellsY = static_cast<int>(_ySize / _cellSize);

        createEmptyGrid();
        _cellUpdateThread = std::thread([this](){ pushUpdatesToGrid(); });
    } // constructor

    GridManager2D::~GridManager2D(){
        _cellUpdateThreadRunning = false;
        if(_cellUpdateThread.joinable())
            _cellUpdateThread.join();
    } // destructor

    std::vector<Cell> GridManager2D::getCells([[maybe_unused]] float time){
        if (initialized){
            // time not used in a static grid
            return _grid;
        }
        else{
            // TODO: implement error handling here -> actual logging for now just return grid
            throw std::runtime_error("Grid not initialized");
        }
    } // getCells

    // TODO: implement multithreaded approach to make grid update
    // e.g updater thread that we push updates to so the thread that holds the grid doesnt risk getting throttled under large data load
    void GridManager2D::createEmptyGrid() {
        _grid.clear();

        for (int i = 0; i < _numCellsX; i++)  {
            for (int j = 0; j < _numCellsY; j++)  {
                float centerX = (i + 0.5) * _cellSize + _xMin;
                float centerY = (j + 0.5) * _cellSize + _yMin;

                _grid.emplace_back(Cell(PointXY(centerX, centerY), IndexXY(i,j), _cellSize));
            }
        }

        initialized = true;
    } // createEmptyGrid

    // returns the cell that contains point at x, y, z
    Cell GridManager2D::getCell(float x, float y) {

        int cellIndexX = static_cast<int>(std::round((x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::round((y - _yMin) / _cellSize));

        if (cellIndexX < 0 || cellIndexX >= _numCellsX || cellIndexY < 0 || cellIndexY >= _numCellsY)    {
            // TODO: implement actuall error handling system -> could log and return -1, -1, -1 just null for now
            // std::cout << "WARNING: attempted access of grid outside of defined domain\n";
            // return a meaningless cell that is listed as an obstacle
            /* TODO: decide of the best way of handling this error
             * construct a cell at that point and label it as an obstacle
             * return meaning;ess occupied cell
             * come up with a create an out of bounds state, construct a Cell of that state and return it, make the class calling this handle it
             */
            return Cell(PointXY(0.0f,0.0f), IndexXY(0,0), _cellSize, State::OBSTACLE,1.0f);
        }

        int calcIndex = cellIndexX + _numCellsX * (cellIndexY + _numCellsY);

        return _grid[calcIndex];
    } // getCell

    // returns the cell that contains PointXYZ point
    Cell GridManager2D::getCell(PointXY point) 
    {
        int cellIndexX = static_cast<int>(std::round((point.x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::round((point.y - _yMin) / _cellSize));

        if (cellIndexX < 0 || cellIndexX >= _numCellsX || cellIndexY < 0 || cellIndexY >= _numCellsY)    {
            // TODO: implement actuall error handling system -> could log and return -1, -1, -1 just null for now
            // std::cout << "WARNING: attempted access of grid outside of defined domain\n";
            // return a meaningless cell that is listed as an obstacle
            /* TODO: decide of the best way of handling this error
             * construct a cell at that point and label it as an obstacle
             * return meaning;ess occupied cell
             * come up with a create an out of bounds state, construct a Cell of that state and return it, make the class calling this handle it
             */
            return Cell(PointXY(0.0f,0.0f), IndexXY(0,0), _cellSize, State::OBSTACLE,1.0f);
        }

        int calcIndex = cellIndexX + _numCellsX * (cellIndexY);

        return _grid[calcIndex];
    } // getCell

    // TODO: implement caching. depends on what hardware this runs on bc if we dont care abt space we could just hold a massive lookup table tbh (doesnt seem wise for a library that could be on many different systems -Drew)
    std::vector<Cell> GridManager2D::getNeighbors(float x, float y, int depth)   
    {
        std::vector<Cell> neighbors;

        int cellIndexX = static_cast<int>(std::round((x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::round((y - _yMin) / _cellSize));
        // TODO: there is a deterministic way to implement this given we know how the list is structured 
        // TODO: error handling? yes and yes --
        int minX = std::max(0, cellIndexX - depth);
        int maxX = std::min(_numCellsX - 1, cellIndexX + depth);
        int minY = std::max(0, cellIndexY - depth);
        int maxY = std::min(_numCellsY - 1, cellIndexY + depth);

        // TODO: no handling for if we are looking at something at an edge or corner of the grid 
        for (int i = minX; i < maxX; i++)   {
            for (int j = minY; j < maxY; j++)   {
                int neighborIndex = x + _numCellsX * (y);
                neighbors.push_back(_grid[neighborIndex]);
            }
        }

        return neighbors;
    } // getNeighbors

    void GridManager2D::addKnownObstacle(float x, float y)  
    {

        int cellIndexX = static_cast<int>(std::round((x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::round((y - _yMin) / _cellSize));

        bool invalid = cellIndexX < 0 || cellIndexX >= _numCellsX || cellIndexY < 0 || cellIndexY >= _numCellsY;

        if (!invalid)   {
            std::scoped_lock lock(_cellUpdateMutex);
            
            int calcIndex = cellIndexX + _numCellsX * (cellIndexY);

            _updates.push_back(UpdateRequest{calcIndex, 1});
        }
    } // addKnownObstacle

    void GridManager2D::addKnownObstacle(const PointXY& point)   {
        int _numCellsX = static_cast<int>(_xSize / _cellSize);
        int _numCellsY = static_cast<int>(_ySize / _cellSize);

        int cellIndexX = static_cast<int>(std::round((point.x - _xMin) / _cellSize));
        int cellIndexY = static_cast<int>(std::round((point.y - _yMin) / _cellSize));

        bool invalid = cellIndexX < 0 || cellIndexX >= _numCellsX || cellIndexY < 0 || cellIndexY >= _numCellsY;

        if (!invalid)   {
            std::scoped_lock lock(_cellUpdateMutex);
            
            int calcIndex = cellIndexX + _numCellsX * (cellIndexY);

            _updates.push_back(UpdateRequest{calcIndex, 1});
        }
    } // addKnownObstacle

    void GridManager2D::addKnownObstacle(std::unique_ptr<I2DObstacle> obstacle)
    {
        // for now this is only handling certain static obstacles
        // TODO: add cases for each child of I2Dobstacle (uncertainones will require monte carlo sim or something PCE??)
        const std::vector<PointXY>& obstaclePolygon = obstacle->getCorners();

        // TODO: optimize
        for(auto& cell : _grid)
        {
            const auto CellPolygon  = cell.getCorners();
            if(Math::Geometry::polygonsIntersect(CellPolygon,obstaclePolygon))
            {
                cell.setState(State::OBSTACLE);
            }
        }
        _obstacleList.push_back(std::move(obstacle));

    } // addKnownObstacle

    void GridManager2D::pushUpdatesToGrid()   {
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

    } // pushUpdatesToGrid

    PointXY GridManager2D::getNearestObstacleCenter(PointXY PointOfReference)
    {
        float lowestDistance = INFINITY;
        int closestCellIndex = -1;
        int currentIndex = 0;
        for(const auto& cell : _grid)
        {
            // Run the checks
            if(cell.getState() == State::OBSTACLE)
            {
                float distance = PointXY(cell.getCenter() - PointOfReference).norm();
                if(distance < lowestDistance)
                {
                    closestCellIndex = currentIndex;
                }
            }
            currentIndex++;
        }

        return _grid.at(closestCellIndex).getCenter();

    } // getNearestObstacleCenter


    void GridManager2D::getGridDomain(float& xMin, float& xMax, float& yMin, float& yMax)
    {
        xMin = _xMin;
        xMax = _xMax;
        yMin = _yMin;
        yMax = _yMax;
    } // getGridDomain

    const float GridManager2D::getCellSize(){
        return _cellSize;
    } // getCellSize

    const IndexXY GridManager2D::getIndexContainingPoint(const PointXY point)
    {
        return IndexXY(static_cast<int>(std::round((point.x + 0.5 * _cellSize) / _cellSize)), static_cast<int>(std::round((point.y + 0.5 * _cellSize) / _cellSize)));
    } // getIndexContainingPoint

    const PointXY GridManager2D::getPointFromIndex(const IndexXY index)
    {
        return PointXY(index.x * _cellSize - 0.5 * _cellSize, index.y * _cellSize - 0.5 * _cellSize);
    } // getPointFromIndex

}


