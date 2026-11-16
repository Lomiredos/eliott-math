#pragma once
#include "Constants.hpp"
#include "Vector.hpp"
#include <algorithm>
#include <cmath>
#include <concepts>

namespace ee::math {

    template <std::floating_point T>
    struct AxisAngle {
        Vector<3, T> axis;
        T angle;
    };

    template <std::floating_point T>
    struct Quaternion : private Vector<4, T> {
        using Vec = Vector<4, T>;
        using Vec::w;
        using Vec::x;
        using Vec::y;
        using Vec::z;

      private:
        constexpr explicit Quaternion(const Vec& _v) noexcept : Vec(_v) {}

        [[nodiscard]] static constexpr Quaternion fromVec(const Vec& _v) noexcept { return Quaternion(_v); }

      public:
        constexpr Quaternion() noexcept : Vec{T(0), T(0), T(0), T(1)} {}
        constexpr Quaternion(T _x, T _y, T _z, T _w) noexcept : Vec{_x, _y, _z, _w} {}

        [[nodiscard]] constexpr const Vec& vec() const noexcept { return *this; }

        // construction
        [[nodiscard]] static constexpr Quaternion identity() noexcept { return {}; }
        [[nodiscard]] static Quaternion fromAxisAngle(const Vector<3, T>& _axis, T _angleRad) noexcept;
        [[nodiscard]] static Quaternion fromEulerDeg(T _rxDeg, T _ryDeg, T _rzDeg) noexcept;
        [[nodiscard]] static Quaternion fromEulerDeg(const Vector<3, T>& _deg) noexcept { return fromEulerDeg(_deg.x, _deg.y, _deg.z); }
        [[nodiscard]] static Quaternion fromTo(const Vector<3, T>& _from, const Vector<3, T>& _to, T _eps = T(1e-5)) noexcept;
        [[nodiscard]] static Quaternion fromBasis(const Vector<3, T>& _right, const Vector<3, T>& _up, const Vector<3, T>& _back) noexcept;
        [[nodiscard]] static Quaternion lookRotation(const Vector<3, T>& _forward, const Vector<3, T>& _up = {T(0), T(1), T(0)}) noexcept;

        // norme
        [[nodiscard]] constexpr T lengthSq() const noexcept { return vec().lengthSq(); }
        [[nodiscard]] T length() const noexcept { return vec().length(); }
        [[nodiscard]] Quaternion normalized() const noexcept { return fromVec(vec().normalized()); }
        Quaternion& normalize() noexcept { return *this = normalized(); }
        [[nodiscard]] constexpr bool isNormalized(T _eps = T(1e-5)) const noexcept {
            const T d = lengthSq() - T(1);
            return d <= _eps && -d <= _eps;
        }

        [[nodiscard]] friend constexpr T dot(const Quaternion& _a, const Quaternion& _b) noexcept { return dot(_a.vec(), _b.vec()); }

        // conversions
        [[nodiscard]] Vector<3, T> toEulerDeg(T _eps = T(1e-5)) const noexcept;
        [[nodiscard]] AxisAngle<T> toAxisAngle(T _eps = T(1e-6)) const noexcept;

        // algebre
        [[nodiscard]] constexpr Quaternion operator+(const Quaternion& _o) const noexcept { return fromVec(vec() + _o.vec()); }
        [[nodiscard]] constexpr Quaternion operator-(const Quaternion& _o) const noexcept { return fromVec(vec() - _o.vec()); }
        [[nodiscard]] constexpr Quaternion operator-() const noexcept { return fromVec(-vec()); }
        [[nodiscard]] constexpr Quaternion operator*(T _s) const noexcept { return fromVec(vec() * _s); }
        [[nodiscard]] constexpr Quaternion operator/(T _s) const noexcept { return fromVec(vec() / _s); }
        [[nodiscard]] friend constexpr Quaternion operator*(T _s, const Quaternion& _q) noexcept { return _q * _s; }

        [[nodiscard]] constexpr Quaternion operator*(const Quaternion& _q) const noexcept;
        [[nodiscard]] constexpr Vector<3, T> operator*(const Vector<3, T>& _v) const noexcept { return rotate(_v); }

        constexpr Quaternion& operator+=(const Quaternion& _o) noexcept { return *this = *this + _o; }
        constexpr Quaternion& operator-=(const Quaternion& _o) noexcept { return *this = *this - _o; }
        constexpr Quaternion& operator*=(const Quaternion& _q) noexcept { return *this = *this * _q; }
        constexpr Quaternion& operator*=(T _s) noexcept { return *this = *this * _s; }
        constexpr Quaternion& operator/=(T _s) noexcept { return *this = *this / _s; }

        [[nodiscard]] constexpr bool operator==(const Quaternion&) const noexcept = default;

        [[nodiscard]] constexpr Quaternion conjugate() const noexcept { return {-x, -y, -z, w}; }
        [[nodiscard]] constexpr Quaternion inverse() const noexcept { return conjugate() / lengthSq(); }

        // rotation
        [[nodiscard]] constexpr Vector<3, T> rotate(const Vector<3, T>& _v) const noexcept;

        [[nodiscard]] constexpr Vector<3, T> forward() const noexcept { return rotate({T(0), T(0), T(-1)}); }
        [[nodiscard]] constexpr Vector<3, T> back() const noexcept { return rotate({T(0), T(0), T(1)}); }
        [[nodiscard]] constexpr Vector<3, T> up() const noexcept { return rotate({T(0), T(1), T(0)}); }
        [[nodiscard]] constexpr Vector<3, T> down() const noexcept { return rotate({T(0), T(-1), T(0)}); }
        [[nodiscard]] constexpr Vector<3, T> right() const noexcept { return rotate({T(1), T(0), T(0)}); }
        [[nodiscard]] constexpr Vector<3, T> left() const noexcept { return rotate({T(-1), T(0), T(0)}); }

        // comparaison
        [[nodiscard]] friend bool sameRotation(const Quaternion& _a, const Quaternion& _b, T _eps = T(1e-5)) noexcept { return std::abs(dot(_a, _b)) >= T(1) - _eps; }

        [[nodiscard]] friend T angleBetween(const Quaternion& _a, const Quaternion& _b) noexcept { return T(2) * std::acos(std::min(T(1), std::abs(dot(_a, _b)))); }

        // interpolation
        [[nodiscard]] friend Quaternion nlerp(const Quaternion& _a, const Quaternion& _b, T _t) noexcept {
            const Vec b = dot(_a, _b) < T(0) ? -_b.vec() : _b.vec();
            return fromVec(lerp(_a.vec(), b, _t)).normalized();
        }

        [[nodiscard]] friend Quaternion slerp(const Quaternion& _a, Quaternion _b, T _t) noexcept {
            T d = dot(_a, _b);
            if (d < T(0)) {
                _b = -_b;
                d = -d;
            }
            if (d > T(0.9995))
                return nlerp(_a, _b, _t);

            const T theta = std::acos(d);
            const T sinTheta = std::sin(theta);
            const T wa = std::sin((T(1) - _t) * theta) / sinTheta;
            const T wb = std::sin(_t * theta) / sinTheta;
            return _a * wa + _b * wb;
        }

        template <EasingFunction<T> F>
        [[nodiscard]] friend Quaternion ease(const Quaternion& _a, const Quaternion& _b, T _t, F _f) noexcept {
            return slerp(_a, _b, static_cast<T>(_f(_t)));
        }

        [[nodiscard]] friend Quaternion rotateTowards(const Quaternion& _from, const Quaternion& _to, T _maxAngle) noexcept {
            const T angle = angleBetween(_from, _to);
            if (angle <= _maxAngle || angle < T(1e-6))
                return _to;
            return slerp(_from, _to, _maxAngle / angle);
        }
    };

    static_assert(sizeof(Quaternion<float>) == 4 * sizeof(float));
    static_assert(std::is_trivially_copyable_v<Quaternion<float>>);

} // namespace ee::math

#include "Quaternion.inl"
