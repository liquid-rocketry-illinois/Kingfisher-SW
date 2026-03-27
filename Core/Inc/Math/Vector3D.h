//
// Created by dyrel on 2/26/2026.
//

#ifndef KINGFISHER_SW_Vector3D_H
#define KINGFISHER_SW_Vector3D_H

#include <cmath>
#include <type_traits>
#include <limits>
#include <stdexcept>

/** @brief a 3D vector class. Vector3D<instantiation type> variable_name(x,y,z);
 *
 * @param T instantiation type. float creates a float vector,
 *          int produces an int vector, etc.
 */
template<typename T>
class Vector3D {
    static_assert(std::is_arithmetic_v<T>,
                  "Vector3D requires arithmetic type");

public:
    T x;
    T y;
    T z;

    using value_type = T;

    // =============================
    // Constructors
    // =============================

    constexpr Vector3D() : x(0), y(0), z(0) {}
    constexpr Vector3D(T X, T Y, T Z) : x(X), y(Y), z(Z) {}

    template<typename U>
    constexpr explicit Vector3D(const Vector3D<U>& v)
        : x(static_cast<T>(v.x)),
          y(static_cast<T>(v.y)),
          z(static_cast<T>(v.z)) {}

    // =============================
    // Type Promotion Helper
    // =============================

    template<typename U>
    using Promote = Vector3D<std::common_type_t<T, U>>;

    // =============================
    // Arithmetic (Mixed Type Safe)
    // =============================

    template<typename U>
    constexpr Promote<U> operator+(const Vector3D<U>& v) const {
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(x) + v.x,
            static_cast<R>(y) + v.y,
            static_cast<R>(z) + v.z
        );
    }

    template<typename U>
    constexpr Promote<U> operator-(const Vector3D<U>& v) const {
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(x) - v.x,
            static_cast<R>(y) - v.y,
            static_cast<R>(z) - v.z
        );
    }

    template<typename U>
    constexpr Promote<U> operator*(U scalar) const {
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(x) * scalar,
            static_cast<R>(y) * scalar,
            static_cast<R>(z) * scalar
        );
    }

    template<typename U>
    constexpr Promote<U> operator/(U scalar) const {
        if (scalar == 0)
            throw std::runtime_error("Division by zero");
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(x) / scalar,
            static_cast<R>(y) / scalar,
            static_cast<R>(z) / scalar
        );
    }

    // =============================
    // Dot & Cross
    // =============================

    template<typename U>
    constexpr auto dot(const Vector3D<U>& v) const {
        using R = std::common_type_t<T,U>;
        return static_cast<R>(x)*v.x +
               static_cast<R>(y)*v.y +
               static_cast<R>(z)*v.z;
    }

    template<typename U>
    constexpr Promote<U> cross(const Vector3D<U>& v) const {
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(y)*v.z - static_cast<R>(z)*v.y,
            static_cast<R>(z)*v.x - static_cast<R>(x)*v.z,
            static_cast<R>(x)*v.y - static_cast<R>(y)*v.x
        );
    }

    // =============================
    // Norms
    // =============================

    auto magnitude() const {
        using R = std::conditional_t<
            std::is_floating_point<T>::value,
            T,
            double>;

        R xx = static_cast<R>(x);
        R yy = static_cast<R>(y);
        R zz = static_cast<R>(z);

        return std::sqrt(xx*xx + yy*yy + zz*zz);
    }

    constexpr auto magnitudeSquared() const {
        using R = std::common_type_t<T,T>;
        return static_cast<R>(x)*x +
               static_cast<R>(y)*y +
               static_cast<R>(z)*z;
    }

    auto normalized() const {
        auto mag = magnitude();
        if (mag == 0)
            throw std::runtime_error("Cannot normalize zero vector");

        using R = decltype(mag);
        return Vector3D<R>(
            static_cast<R>(x) / mag,
            static_cast<R>(y) / mag,
            static_cast<R>(z) / mag
        );
    }

    // =============================
    // Distance
    // =============================

    template<typename U>
    auto distance(const Vector3D<U>& v) const {
        return (*this - v).magnitude();
    }

    // =============================
    // Safe Float Comparison
    // =============================

    bool isApprox(const Vector3D& v,
                  T eps = std::numeric_limits<T>::epsilon()*100) const {
        if constexpr (std::is_floating_point<T>::value) {
            return std::fabs(x - v.x) < eps &&
                   std::fabs(y - v.y) < eps &&
                   std::fabs(z - v.z) < eps;
        } else {
            return *this == v;
        }
    }

    constexpr bool operator==(const Vector3D& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    constexpr bool operator!=(const Vector3D& v) const {
        return !(*this == v);
    }
};

// Scalar left multiply
template<typename T, typename U>
constexpr auto operator*(T scalar, const Vector3D<U>& v) {
    return v * scalar;
}

#endif //KINGFISHER_SW_Vector3D_H