#include <iostream>
#include <vector>
#include "GridTypes.hpp"

namespace lamb::cpp::threedee {

class GridManager   {
    public:
    GridManager(float p_maxX, float p_maxY, float p_maxZ, float p_cellSize);

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

    
    private:
    std::vector<Cell> _grid;

    float cellSize;
    float xSize;
    float ySize;
    float zSize;

    bool initialized = false;

    void createEmptyGrid(float p_maxX, float p_maxY, float p_maxZ, float p_cellSize);

    

};
}