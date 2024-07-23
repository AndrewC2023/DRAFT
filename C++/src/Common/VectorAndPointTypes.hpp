/*
 * Author: Andrew Campbell
 * Date: 07-21-2024
 */

#ifndef VECTORANDPOINTS_H
#define VECTORANDPOINTS_H

// STL
#include <iostream>
#include <cmath>
#include <string>

namespace astro::platform_code::common::custom_types
{
    /** Implementation of some basic structures to use for navigation. Previously, objects like Eigen::Vector3f were used,
     *  but the implementation of those was not very efficient. These are simple structures with 2 or 3 templated data fields
     *  that can be applied to many of our use cases, along with commonly used operators that provide compatibility with most
     *  of the functions of their predecessors.
     *
     *  The Eigen::Vector3f class was used in many cases for pose X/Y/Theta, or other similar cases. The Eigen implementation is
     *  based off their Matrix object, and therefore was incredibly slow in our applications, especially when performing many
     *  accesses as in navigation. These structures can be use in the same way as the Eigen implementations, but do not suffer
     *  from the same problems.
     *
     * @tparam T the data type of the objects
     */
    template<typename T>
    struct vector2
    {
        vector2() noexcept : x(0), y(0) {};
        vector2(T& x, T& y) noexcept : x(x), y(y) {};
        vector2(const T& x, const T& y) noexcept :  x(x), y(y) {};
        vector2(vector2& other) noexcept : x(other.x), y(other.y) {};
        vector2(const vector2& other) noexcept : x(other.x), y(other.y) {};
        vector2(const vector2&& other) noexcept : x(other.x), y(other.y) {}; // why????
        ~vector2() = default;

        vector2& operator=(const vector2& other) noexcept
        {
            x = other.x;
            y = other.y;
            return *this;
        };
        vector2& operator=(vector2&& other) noexcept
        {
            x = other.x;
            y = other.y;
            return *this;
        };

        bool operator==(vector2& other) const { return x == other.x && y == other.y; };
        bool operator==(const vector2& other) const { return x == other.x && y == other.y; };
        bool operator!=(vector2& other) const { return x != other.x || y != other.y; };
        bool operator!=(const vector2& other) const { return x != other.x || y != other.y; };

        // Vector addition
        vector2 operator+(vector2& other) const { return vector2(x + other.x, y + other.y); }
        vector2 operator+(const vector2& other) const { return vector2(x + other.x, y + other.y); }
        vector2 operator-(vector2& other) const { return vector2(x - other.x, y - other.y); }
        vector2 operator-(const vector2& other) const { return vector2(x - other.x, y - other.y); }
        
        // Scalar operations
        vector2 operator*(const float n) const { return vector2(x * n, y * n); }
        vector2 operator*(const double n) const { return vector2(x * n, y * n); }
        vector2 operator*(const int n) const { return vector2(x * n, y * n); }
        vector2 operator/(const float n) const { return vector2(x / n, y / n); }
        vector2 operator/(const double n) const { return vector2(x / n, y / n); }
        vector2 operator/(const int n) const { return vector2(x / n, y / n); }

        // dot product (abstracted for clarity)
        T dot(const vector2& other) const { return vector2(x * other.x, y * other.y); }

        [[nodiscard]] double norm() const { return static_cast<double>(std::sqrt(x * x + y * y)); }

        friend std::ostream& operator<<(std::ostream& stream, const vector2& data)
        {
            stream << "( " << data.x << ", " << data.y << " )";
            return stream;
        };

        [[nodiscard]] std::string string() const {
            return std::string("( " + std::to_string(x) + ", " + std::to_string(y)  + " )");
        }

        T x;
        T y;
    };

    template<typename T>
    struct vector3
    {
        vector3() noexcept : x(0), y(0), z(0) {};
        vector3(T& x, T& y, T& z) noexcept : x(x), y(y), z(z) {};
        vector3(const T& x, const T& y, const T& z) noexcept : x(x), y(y), z(z) {};
        vector3(vector3& other) noexcept : x(other.x), y(other.y), z(other.z) {};
        vector3(const vector3& other) noexcept : x(other.x), y(other.y), z(other.z) {};
        vector3(const vector3&& other) noexcept : x(other.x), y(other.y), z(other.z) {};
        ~vector3() = default;

        vector3& operator=(const vector3& other) noexcept
        {
            x = other.x;
            y = other.y;
            z = other.z;
            return *this;
        };
        vector3& operator=(vector3&& other) noexcept
        {
            x = other.x;
            y = other.y;
            z = other.z;
            return *this;
        };

        bool operator==(vector3& other) const { return x == other.x && y == other.y && z == other.z; };
        bool operator==(const vector3& other) const { return x == other.x && y == other.y && z == other.z; };
        bool operator!=(vector3& other) const { return x != other.x || y != other.y || z != other.z; };
        bool operator!=(const vector3& other) const { return x != other.x || y != other.y || z != other.z; };

        vector3 operator+(vector3& other) const { return vector3(x + other.x, y + other.y, z + other.z); }
        vector3 operator+(const vector3& other) const { return vector3(x + other.x, y + other.y, z + other.z); }
        vector3 operator-(vector3& other) const { return vector3(x - other.x, y - other.y, z - other.z); }
        vector3 operator-(const vector3& other) const { return vector3(x - other.x, y - other.y, z - other.z); }
        vector3 operator*(vector3& other) const { return vector3(x * other.x, y * other.y, z * other.z); }
        vector3 operator*(const vector3& other) const { return vector3(x * other.x, y * other.y, z * other.z); }
        vector3 operator/(vector3& other) const { return vector3(x / other.x, y / other.y, z / other.z); }
        vector3 operator/(const vector3& other) const { return vector3(x / other.x, y / other.y, z / other.z); }

        vector3 operator+(const float n) const { return vector3(x + n, y + n, z + n); }
        vector3 operator+(const double n) const { return vector3(x + n, y + n, z + n); }
        vector3 operator+(const int n) const { return vector3(x + n, y + n, z + n); }
        vector3 operator-(const float n) const { return vector3(x - n, y - n, z - n); }
        vector3 operator-(const double n) const { return vector3(x - n, y - n, z - n); }
        vector3 operator-(const int n) const { return vector3(x - n, y - n, z - n); }
        vector3 operator*(const float n) const { return vector3(x * n, y * n, z * n); }
        vector3 operator*(const double n) const { return vector3(x * n, y * n, z * n); }
        vector3 operator*(const int n) const { return vector3(x * n, y * n, z * n); }
        vector3 operator/(const float n) const { return vector3(x / n, y / n, z / n); }
        vector3 operator/(const double n) const { return vector3(x / n, y / n, z / n); }
        vector3 operator/(const int n) const { return vector3(x / n, y / n, z / n); }

        [[nodiscard]] double norm() const { return static_cast<double>(std::sqrt(x * x + y * y + z * z)); }

        friend std::ostream& operator<<(std::ostream& stream, const vector3& data)
        {
            stream << "( " << data.x << ", " << data.y << ", " << data.z << " )";
            return stream;
        };

        [[nodiscard]] std::string string() const {
            return std::string("( " + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + " )");
        }

        T x;
        T y;
        T z;
    };

    /// Macros with same name as Eigen types
    typedef vector2<int> Vector2i;
    typedef vector2<float> Vector2f;
    typedef vector2<double> Vector2d;

    typedef vector3<int> Vector3i;
    typedef vector3<float> Vector3f;
    typedef vector3<double> Vector3d;
}

/// Macros for use in navigation
namespace astro::autonomy::navigation
{
    typedef astro::platform_code::common::custom_types::vector2<std::size_t> IndexXY;         // Index type, with X and Y indices
    typedef astro::platform_code::common::custom_types::vector2<float> PointXY;               // Point type with floating point X and Y coordinates
    typedef astro::platform_code::common::custom_types::vector3<float> PointXYZ;              // Point type with floating point X, Y and Z coordinates
    typedef astro::platform_code::common::custom_types::vector2<float> PolarCoordinateRT;     // Polar coordinate with length r and theta t
}

/// Macros for use in HMI
namespace astro::operator_interface::hmi
{
    typedef astro::platform_code::common::custom_types::vector3<float> ColorRGB;
}

#endif // VECTORANDPOINTS_H