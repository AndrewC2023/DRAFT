/*
 * Author: Andrew Campbell
 * Date: 07-25-2024
 */

#include "CustomTypes/VectorAndPointTypes.hpp"

#include <iostream>

int main()
{
    Util::Custom_Types::Vectors::AnglesRollPitchYaw thingy(0.0f,0.0f,0.0f);
    std::cout << thingy.x;

    return 0;
}