#include <iostream>
#include <cmath>
#include <vector>

struct PointXYZ {
    float x;
    float y;
    float z; 
};

enum State  {
    CLEAR,
    OBSTACLE
};

class Cell {
    public:
        Cell(PointXYZ p_center)   {
            _center = p_center;
            //TODO: calculate corners
        }

        PointXYZ getCenter()    {
            return _center;
        }

        std::vector<int> getCorners()   {
            return _corners;
        }

        State getState()    {
            return _state;
        }


    private:
        PointXYZ _center;
        std::vector<int> _corners;
        State _state;

};