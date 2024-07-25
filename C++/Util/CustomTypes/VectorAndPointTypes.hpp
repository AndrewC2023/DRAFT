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

namespace Util::Custom_Types::Vectors
{
    /** Implementation of some basic structures to use for planning. objects like Eigen::Vector3f can be used,
     *  but the implementation of those are not very efficient. These are simple structures with 2 or 3 templated data fields
     *  that can be applied to many of our use cases, along with commonly used operators that provide compatibility with most
     *  of the functions of their predecessors.
     *
     *  The Eigen::Vector3f class was used in many cases for pose X/Y/Theta, or other similar cases. The Eigen implementation is
     *  based off their Matrix object, and therefore is incredibly slow for applications that do not require all the functionality 
     *  an eigen library matrix provides, especially when performing many accesses as in navigation.
     *  These structures can be use in the same way as the Eigen implementations, but do not suffer from the same problems.
     *
     *  TODO: if this is to be ported over to or become its own library then a way to interface these types with more standard types or structures that someone may use could be necessary
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
        T dot(const vector2& other) const { return ((x * other.x) + (y * other.y)); }


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
        
        // Vector addition
        vector3 operator+(vector3& other) const { return vector3(x + other.x, y + other.y, z + other.z); }
        vector3 operator+(const vector3& other) const { return vector3(x + other.x, y + other.y, z + other.z); }
        vector3 operator-(vector3& other) const { return vector3(x - other.x, y - other.y, z - other.z); }
        vector3 operator-(const vector3& other) const { return vector3(x - other.x, y - other.y, z - other.z); }
    
        // Scalar operations
        vector3 operator*(const float n) const { return vector3(x * n, y * n, z * n); }
        vector3 operator*(const double n) const { return vector3(x * n, y * n, z * n); }
        vector3 operator*(const int n) const { return vector3(x * n, y * n, z * n); }
        vector3 operator/(const float n) const { return vector3(x / n, y / n, z / n); }
        vector3 operator/(const double n) const { return vector3(x / n, y / n, z / n); }
        vector3 operator/(const int n) const { return vector3(x / n, y / n, z / n); }

        // dot product
        T dot(const vector3& other) const { return ((x * other.x) + (y * other.y) + (z * other.z)); }
        // cross product
        vector3 cross(const vector3& other) const {return vector3((y * other.z) - (z * other.y), (z * other.x) - (x * other.z), (x * other.y) - (y * other.x)); }


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

    template<typename T>
    struct angle3 : vector3<T>
    {
        using vector3<T>::vector3;
        
        union {
            struct { T roll, pitch, yaw; };
            struct { T x, y, z; };
        };

        angle3() noexcept : roll(0), pitch(0), yaw(0) {};
        angle3(T& x, T& y, T& z) noexcept : roll(x), pitch(y), yaw(z) {};
        angle3(const T& x, const T& y, const float& z) noexcept : roll(x), pitch(y), yaw(z) {};
        angle3(vector3& other) noexcept : roll(other.x), pitch(other.y), yaw(other.z) {};
        angle3(const vector3& other) noexcept : x(other.x), y(other.y), z(other.z) {};
        angle3(const vector3&& other) noexcept : x(other.x), y(other.y), z(other.z) {};
        ~angle3() = default;

        private:
            using vector3<T>::x;
    };
    
    /// Macros with same name as Eigen types
    typedef vector2<int> Vector2i;
    typedef vector2<float> Vector2f;
    typedef vector2<double> Vector2d;

    typedef vector3<int> Vector3i;
    typedef vector3<float> Vector3f;
    typedef vector3<double> Vector3d;

    typedef angle3<float> AnglesRollPitchYaw;
}

/// Macros for use in navigation
namespace Algorithms::TwoD
{
    typedef Util::Custom_Types::Vectors::vector2<int> IndexXY;                 // Index type, with X and Y indices
    typedef Util::Custom_Types::Vectors::vector2<float> PointXY;               // Point type with floating point X and Y coordinates
    typedef Util::Custom_Types::Vectors::vector2<float> PolarCoordinateRT;     // Polar coordinate with length r and theta t
    typedef Util::Custom_Types::Vectors::vector3<float> StateXYT;              // 2D state of XY and Theta for heading
}

namespace Algorithms::ThreeD
{
    typedef Util::Custom_Types::Vectors::vector3<int> IndexXYZ;                // Index type for 3D grid manager
    typedef Util::Custom_Types::Vectors::vector3<float> PointXYZ;              // XYZ Point
    typedef Util::Custom_Types::Vectors::vector3<float> AnglesRollPitchYaw;    // Roll Pitch Yaw angles
    typedef Util::Custom_Types::Vectors::vector3<float> CylindricalRhoPhiZ;    // Cylindrical Coordinates
    typedef Util::Custom_Types::Vectors::vector3<float> SphericalRThPhi;       // Shperical Coordinates 
}

namespace Util::Custom_Types::Vectors
{

    /** 3D state vector
     *  Defines for an easy way to manage and manipulate the 6 dof state of a three dimensional body potentially useful for planners/paths in three dimensions
     * 
     *  @warning This should not be used for control sytems  or state matricies/vectors in general as those should be constructed 
     * as eigen library objects for their extra functionality
     * 
     */
    template<typename T>
    struct statevector6dof
    {
        statevector6dof() noexcept : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {};
        statevector6dof(T& x, T& y, T& z, T& roll, T& pitch, T& yaw) noexcept : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {};
        statevector6dof(const T& x, const T& y, const T& z) noexcept : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {};
        statevector6dof(statevector6dof& other) noexcept : x(other.x), y(other.y), z(other.z), roll(other.roll), pitch(other.pitch), yaw(other.yaw) {};
        statevector6dof(const statevector6dof& other) noexcept : x(other.x), y(other.y), z(other.z), roll(other.roll), pitch(other.pitch), yaw(other.yaw) {};
        statevector6dof(const statevector6dof&& other) noexcept : x(other.x), y(other.y), z(other.z), roll(other.roll), pitch(other.pitch), yaw(other.yaw) {};
        statevector6dof(Algorithms::ThreeD::PointXYZ& points, Algorithms::ThreeD::AnglesRollPitchYaw angles) noexcept : x(points.x), y(points.y), z(points.z), roll(angles.roll), pitch(other.pitch), yaw(other.yaw) {};
        ~statevector6dof() = default;

        statevector6dof& operator=(const statevector6dof& other) noexcept
        {
            x = other.x;
            y = other.y;
            z = other.z;
            roll = other.roll;
            pitch = other.pitch;
            yaw = other.yaw;
            return *this;
        };
        statevector6dof& operator=(statevector6dof&& other) noexcept
        {
            x = other.x;
            y = other.y;
            z = other.z;
            roll = other.roll;
            pitch = other.pitch;
            yaw = other.yaw;
            return *this;
        };

        bool operator==(statevector6dof& other) const { return x == other.x && y == other.y && z == other.z && roll == other.roll && pitch == other.pitch && yaw == other.yaw; };
        bool operator==(const statevector6dof& other) const { return x == other.x && y == other.y && z == other.z && roll == other.roll && pitch == other.pitch && yaw == other.yaw; };
        bool operator!=(statevector6dof& other) const { return x != other.x || y != other.y || z != other.z || roll != other.roll || pitch != other.pitch || yaw == other.yaw; };
        bool operator!=(const statevector6dof& other) const { return x != other.x || y != other.y || z != other.z || roll != other.roll || pitch != other.pitch || yaw == other.yaw; };
        
        // Vector addition
        statevector6dof operator+(statevector6dof& other) const { return statevector6dof(x + other.x, y + other.y, z + other.z, roll + other.roll, pitch + other.pitch, yaw + other.yaw); }
        statevector6dof operator+(const statevector6dof& other) const { return statevector6dof(x + other.x, y + other.y, z + other.z, roll + other.roll, pitch + other.pitch, yaw + other.yaw); }
        statevector6dof operator+(Algorithms::ThreeD::PointXYZ& other) const {return statevector6dof(x + other.x, y + other.y, z + other.z, roll, pitch, yaw); }
        statevector6dof operator+(const Algorithms::ThreeD::PointXYZ& other) const {return statevector6dof(x + other.x, y + other.y, z + other.z, roll, pitch, yaw); }
        statevector6dof operator+(Algorithms::ThreeD::AnglesRollPitchYaw& other) const {return statevector6dof(x, y, z, roll + other.roll, pitch + other.pitch, yaw + other.yaw); }
        statevector6dof operator+(const Algorithms::ThreeD::AnglesRollPitchYaw& other) const {return statevector6dof(x, y, z, roll + other.roll, pitch + other.pitch, yaw + other.yaw); }
        statevector6dof operator-(statevector6dof& other) const { return statevector6dof(x - other.x, y - other.y, z - other.z, roll - other.roll, pitch - other.pitch, yaw - other.yaw); }
        statevector6dof operator-(const statevector6dof& other) const { return statevector6dof(x - other.x, y - other.y, z - other.z, roll - other.roll, pitch - other.pitch, yaw - other.yaw); }
        statevector6dof operator-(Algorithms::ThreeD::PointXYZ& other) const {return statevector6dof(x - other.x, y - other.y, z - other.z, roll, pitch, yaw); }
        statevector6dof operator-(const Algorithms::ThreeD::PointXYZ& other) const {return statevector6dof(x - other.x, y - other.y, z - other.z, roll, pitch, yaw); }
        statevector6dof operator-(Algorithms::ThreeD::AnglesRollPitchYaw& other) const {return statevector6dof(x, y, z, roll - other.roll, pitch - other.pitch, yaw - other.yaw); }
        statevector6dof operator-(const Algorithms::ThreeD::AnglesRollPitchYaw& other) const {return statevector6dof(x, y, z, roll - other.roll, pitch - other.pitch, yaw - other.yaw); }

        [[nodiscard]] double norm() const { return static_cast<double>(std::sqrt(x * x + y * y + z * z)); }

        friend std::ostream& operator<<(std::ostream& stream, const statevector6dof& data)
        {
            stream << "state 6dof:\n (X: " << data.x << ", Y: " << data.y << ", Z: " << data.z << " )\n(Roll: " << data.roll << ", Pitch: " << data.pitch << ", Yaw: " << data.yaw << " )";
            return stream;
        };

        [[nodiscard]] std::string string() const {
            return std::string("state 6dof:\n (X: " + std::to_string(x) + ", Y: " + std::to_string(y) + ", Z: " + std::to_string(z) + " )\n(Roll: " + std::to_string(roll) + ", Pitch: " + std::to_string(pitch) + ", Yaw: " + std::to_string(yaw) + " )");
        }

        T x;
        T y;
        T z;
        T roll;
        T pitch;
        T yaw;
    }; 
}

namespace Algorithms::ThreeD
{
    // TODO: duh
}

#endif // VECTORANDPOINTS_H