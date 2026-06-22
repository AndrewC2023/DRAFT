/*
 * Author: Andrew Campbell
 * Date: 10-15-2024
 */

#ifndef MATH_H
#define MATH_H

// Eigen
#include <Eigen/Dense>

// STL
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

namespace Draft::Util::Math
{
    /**
     * @brief Converts an angle from degrees to radians.
     *
     * @tparam Tfloat Floating-point type used for the angle.
     * @param angle Angle in degrees.
     * @return Angle in radians.
     */
    template<typename Tfloat>
    Tfloat Degrees2Radians(Tfloat angle) {
        return static_cast<Tfloat>((angle/180)*M_PI);
    };

    /**
     * @brief Converts an angle from radians to degrees.
     *
     * @tparam Tfloat Floating-point type used for the angle.
     * @param angle Angle in radians.
     * @return Angle in degrees.
     */
    template<typename Tfloat>
    Tfloat Radians2Degrees(Tfloat angle) {
        return static_cast<Tfloat>((angle/M_PI)*180);
    };

    /**
     * @brief Wraps an angle to the interval [-pi, pi).
     *
     * @tparam Tfloat Floating-point type used for the angle.
     * @param angle Angle in radians.
     * @return Equivalent angle in the interval [-pi, pi).
     */
    template<typename Tfloat>
    Tfloat WrapAngleToPi(Tfloat angle) {
        while(angle >= M_PI){
            angle -= static_cast<Tfloat>(2.0 * M_PI);
        }
        while (angle < -M_PI) {
            angle += static_cast<Tfloat>(2.0 * M_PI);
        }

        return angle;
    };

    /**
     * @brief Limits a scalar value to a closed interval.
     *
     * @tparam Tvalue Comparable scalar type.
     * @param value Value to limit.
     * @param minValue Minimum allowed value.
     * @param maxValue Maximum allowed value.
     * @return The value limited to [minValue, maxValue].
     * @throws std::invalid_argument If minValue is greater than maxValue.
     */
    template<typename Tvalue>
    Tvalue Saturate(Tvalue value, Tvalue minValue, Tvalue maxValue) {
        if (minValue > maxValue){
            throw std::invalid_argument("Minimum saturation value must not exceed maximum saturation value.");
        }

        return std::clamp(value, minValue, maxValue);
    };

    /**
     * @brief Verifies that every element of a vector is finite.
     *
     * @param vector Vector to validate.
     * @param vectorName Name included in an exception message.
     * @throws std::invalid_argument If any element is NaN or infinite.
     */
    inline void ValidateFinite(const Eigen::VectorXd& vector, const std::string& vectorName) {
        if (!vector.allFinite()){
            throw std::invalid_argument(vectorName + " contains a non-finite value.");
        }
    };

    /**
     * @brief Verifies that every element of a matrix is finite.
     *
     * @param matrix Matrix to validate.
     * @param matrixName Name included in an exception message.
     * @throws std::invalid_argument If any element is NaN or infinite.
     */
    inline void ValidateFinite(const Eigen::MatrixXd& matrix, const std::string& matrixName) {
        if (!matrix.allFinite()){
            throw std::invalid_argument(matrixName + " contains a non-finite value.");
        }
    };

    /**
     * @brief Verifies that a vector has the expected number of elements.
     *
     * @param vector Vector to validate.
     * @param expectedSize Required number of elements.
     * @param vectorName Name included in an exception message.
     * @throws std::invalid_argument If the vector dimension is incorrect.
     */
    inline void ValidateVectorDimension(const Eigen::VectorXd& vector, Eigen::Index expectedSize, const std::string& vectorName) {
        if (vector.size() != expectedSize){
            throw std::invalid_argument(vectorName + " has an invalid dimension.");
        }
    };

    /**
     * @brief Verifies that a matrix has the expected dimensions.
     *
     * @param matrix Matrix to validate.
     * @param expectedRows Required number of rows.
     * @param expectedColumns Required number of columns.
     * @param matrixName Name included in an exception message.
     * @throws std::invalid_argument If either matrix dimension is incorrect.
     */
    inline void ValidateMatrixDimensions(const Eigen::MatrixXd& matrix, Eigen::Index expectedRows, Eigen::Index expectedColumns, const std::string& matrixName) {
        if (matrix.rows() != expectedRows || matrix.cols() != expectedColumns){
            throw std::invalid_argument(matrixName + " has invalid dimensions.");
        }
    };

    /**
     * @brief Limits each vector element to its corresponding closed interval.
     *
     * @param vector Vector containing the values to limit.
     * @param minValues Element-wise minimum allowed values.
     * @param maxValues Element-wise maximum allowed values.
     * @return Saturated vector with the same dimension as the input vector.
     * @throws std::invalid_argument If dimensions differ, an input contains a
     * non-finite value, or any minimum exceeds its corresponding maximum.
     */
    inline Eigen::VectorXd Saturate(const Eigen::VectorXd& vector, const Eigen::VectorXd& minValues, const Eigen::VectorXd& maxValues) {
        ValidateVectorDimension(minValues, vector.size(), "Minimum saturation vector");
        ValidateVectorDimension(maxValues, vector.size(), "Maximum saturation vector");
        ValidateFinite(vector, "Saturation input vector");
        ValidateFinite(minValues, "Minimum saturation vector");
        ValidateFinite(maxValues, "Maximum saturation vector");

        if ((minValues.array() > maxValues.array()).any()){
            throw std::invalid_argument("Minimum saturation values must not exceed maximum saturation values.");
        }

        return vector.cwiseMax(minValues).cwiseMin(maxValues);
    };

    /**
     * @brief Generates a uniformly distributed random value in [0, 1).
     *
     * @return Pseudorandom floating-point value greater than or equal to zero
     * and less than one.
     */
    inline float rand(){
        static std::mt19937 randomGenerator {std::random_device{}()};
        static std::uniform_real_distribution<float> Var(0, 1);
        return Var(randomGenerator);
    }
    
    // TODO: add max function for arrays and lists

    /**
     * @brief Advances a continuous state with one fixed-step classical RK4 update.
     *
     * The derivative callable must accept a time and state vector and return a
     * state derivative vector with the same dimension as the supplied state.
     *
     * @tparam DerivativeFunction Callable type used to evaluate the state derivative.
     * @param time Time associated with the supplied state.
     * @param state State at the beginning of the integration step.
     * @param dt Positive integration step size.
     * @param derivative Callable with signature compatible with
     * `Eigen::VectorXd(double, const Eigen::VectorXd&)`.
     * @return State propagated from time to time + dt.
     * @throws std::invalid_argument If dt is non-finite or not positive.
     * @throws std::runtime_error If a derivative vector dimension does not
     * match the state dimension.
     */
    template<typename DerivativeFunction>
    Eigen::VectorXd RungeKutta4(double time, const Eigen::VectorXd& state, double dt, DerivativeFunction&& derivative)
    {
        if (!std::isfinite(dt) || dt <= 0.0){
            throw std::invalid_argument(
                "RK4 timestep must be finite and positive."
            );
        }

        const Eigen::VectorXd k1 = derivative(time, state);
        const Eigen::VectorXd k2 = derivative(time + 0.5 * dt, state + 0.5 * dt * k1);
        const Eigen::VectorXd k3 = derivative(time + 0.5 * dt, state + 0.5 * dt * k2);
        const Eigen::VectorXd k4 = derivative(time + dt, state + dt * k3);

        if (k1.size() != state.size() || k2.size() != state.size() || k3.size() != state.size() || k4.size() != state.size()){
            throw std::runtime_error(
                "Derivative dimension does not match state dimension."
            );
        }

        return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
}

#endif // MATH_H
