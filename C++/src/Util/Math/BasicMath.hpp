/*
 * Author: Andrew Campbell
 * Date: 10-15-2024
 */

// Custom types
#include "Util/CustomTypes/VectorAndPointTypes.hpp"

// STL
#include <cmath>
#include <iostream>
#include <random>

namespace Math
{
    template<typename Tfloat>
    Tfloat Degrees2Radians(Tfloat angle) {
        return static_cast<Tfloat>((angle/180)*M_PI);
    };

    template<typename Tfloat>
    Tfloat Radians2Degrees(Tfloat angle) {
        return static_cast<Tfloat>((angle/M_PI)*180);
    };

    /// returns a float value between 0 and 1
    float rand(){
        std::mt19937 gen {std::random_device{}()};
        std::uniform_real_distribution<float> Var(0, 1);
        return Var(gen);
    };

}