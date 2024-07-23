#include <iostream>
#include <cmath>
#include <vector>

struct PointXYZ {
    float x;
    float y;
    float z; 
};

struct UpdateRequest    {
    int index;
    float oddsUpdate;
};

enum State  {
    CLEAR,
    UNCERTAIN,
    OBSTACLE
};

class Cell {
    public:
        Cell(PointXYZ p_center, float cellSize)   {
            _center = p_center;
            //TODO: calculate corners
            _corners{}

        }

        PointXYZ getCenter()    {
            return _center;
        }

        std::vector<float> getCorners()   {
            return _corners;
        }

        State getState()    {
            return _state;
        }
        void setState(State p_state) {
            _state = p_state;
        }

        void incrementOdds(int increment) {
            odds += increment;
        }

        float getOdds() {
            return (1 - (1/(1-(std::pow(2, odds)))));
        }


    private:
        PointXYZ _center;
        std::vector<PointXYZ> _corners;
        State _state;
        float odds;

};