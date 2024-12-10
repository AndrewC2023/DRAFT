/*
 * Author: Andrew Campbell
 * Date: 10-15-2024
 */

#ifndef LINEARALGEBRA_H
#define LINEARALGEBRA_H

#include <eigen3/Eigen/Dense>
#include <cmath>

// custom types
#include "Util/CustomTypes/GridTypes2D.hpp"

namespace Math
{
    // DCM matrix 2D
    Eigen::Matrix2f DCM2D(float theta)
    {
        Eigen::Matrix2f DCM(2,2);
        DCM(1,1) = std::cos(theta);
        DCM(1,2) = std::sin(theta);
        DCM(2,1) = (-1)*std::sin(theta);
        DCM(2,2) = std::cos(theta);
        
        return DCM;
    }

    Algorithms::TwoD::PointXY rotate2DPoint(PointXY point, float theta)
    {
        Eigen::Matrix2f DCM = DCM2D(theta);
        Eigen::Vector2f pointVec(point.x, point.y);
        Eigen::Vector2f rotatedVec = DCM * pointVec;
        return Algorithms::TwoD::PointXY(rotatedVec(0), rotatedVec(1));
    }

    // DCM matrix 3D
    Eigen::Matrix3f DCM3D(float theta_x, float theta_y, float theat_z)
    {
        Eigen::Matrix3f DCM(3,3);
        DCM(1,1) = std::cos(theta_y)*std::cos(theat_z);
        DCM(1,2) = std::cos(theta_y)*std::sin(theat_z);
        DCM(1,3) = (-1)*std::sin(theta_y);

        DCM(2,1) = std::sin(theta_x)*std::sin(theta_y)*std::cos(theat_z) - std::cos(theta_x)*std::sin(theat_z);
        DCM(2,2) = std::sin(theta_x)*std::sin(theta_y)*std::sin(theat_z) + std::cos(theta_x)*std::cos(theat_z);
        DCM(2,3) = std::sin(theta_x)*std::cos(theta_y);

        DCM(3,1) = std::cos(theta_x)*std::sin(theta_y)*std::cos(theat_z) + std::sin(theta_x)*std::sin(theat_z);
        DCM(3,2) = std::cos(theta_x)*std::sin(theta_y)*std::sin(theat_z) - std::sin(theta_x)*std::cos(theat_z);
        DCM(3,3) = std::cos(theta_x)*std::cos(theta_y);
        
        return DCM;
    }

}

#endif //LINEARALGEBRA_H
