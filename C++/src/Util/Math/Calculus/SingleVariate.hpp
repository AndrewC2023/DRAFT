/*
 *Author: Andrew Campbell
 * Date: 10-21-2024
 */

#ifndef SINGLEVARIATE_H
#define SINGLEVARIATE_H

#include <functional>
#include <cmath>
namespace Math::Calculus
{   
    template<typename T>
    std::function<T(T)> f_ofX;

    template<typename T>
    float numDerivative(f_ofX<T> func, T evaluationPoint) 
    {
        
    }

    template<typename T>
    float numIntegral(f_ofX<T> func, T lowerBound, T upperBound, int numberDivisions)
    {

    }

    template<typename T>
    float numIntegral(f_ofX<T> func, T lowerBound, T upperBound)
    {
        
    }
}

#endif // SINGLEVARIATE_H