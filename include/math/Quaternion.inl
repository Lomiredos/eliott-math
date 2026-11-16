#pragma once
#include "Quaternion.hpp"

namespace ee::math {

    template <std::floating_point T>
    Quaternion<T> Quaternion<T>::fromAxisAngle(const Vector<3, T>& _axis, T _angleRad) noexcept {
        const Vector<3, T> a = _axis.normalized();
        const T half = _angleRad * T(0.5);
        const T s = std::sin(half);
        return {a.x * s, a.y * s, a.z * s, std::cos(half)};
    }

    template <std::floating_point T>
    Quaternion<T> Quaternion<T>::fromEulerDeg(T _rxDeg, T _ryDeg, T _rzDeg) noexcept {
        const Quaternion qx = fromAxisAngle({T(1), T(0), T(0)}, _rxDeg * constants::degToRad<T>);
        const Quaternion qy = fromAxisAngle({T(0), T(1), T(0)}, _ryDeg * constants::degToRad<T>);
        const Quaternion qz = fromAxisAngle({T(0), T(0), T(1)}, _rzDeg * constants::degToRad<T>);
        return qz * qy * qx;
    }

    template <std::floating_point T>
    Quaternion<T> Quaternion<T>::fromTo(const Vector<3, T>& _from, const Vector<3, T>& _to, T _eps) noexcept {
        const Vector<3, T> f = _from.normalized();
        const Vector<3, T> t = _to.normalized();
        const T cosTheta = dot(f, t);

        if (cosTheta < T(-1) + _eps) {
            Vector<3, T> ortho = cross(Vector<3, T>{T(1), T(0), T(0)}, f);
            if (ortho.lengthSq() < _eps)
                ortho = cross(Vector<3, T>{T(0), T(1), T(0)}, f);
            ortho.normalize();
            return {ortho.x, ortho.y, ortho.z, T(0)};
        }

        const Vector<3, T> axis = cross(f, t);
        return Quaternion{axis.x, axis.y, axis.z, T(1) + cosTheta}.normalized();
    }

    template <std::floating_point T>
    Quaternion<T> Quaternion<T>::fromBasis(const Vector<3, T>& _right, const Vector<3, T>& _up, const Vector<3, T>& _back) noexcept {
        const T m00 = _right.x, m01 = _up.x, m02 = _back.x;
        const T m10 = _right.y, m11 = _up.y, m12 = _back.y;
        const T m20 = _right.z, m21 = _up.z, m22 = _back.z;

        const T trace = m00 + m11 + m22;
        if (trace > T(0)) {
            const T qw = T(0.5) * std::sqrt(T(1) + trace);
            const T s = T(0.25) / qw;
            return {(m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, qw};
        }
        if (m00 >= m11 && m00 >= m22) {
            const T qx = T(0.5) * std::sqrt(T(1) + m00 - m11 - m22);
            const T s = T(0.25) / qx;
            return {qx, (m01 + m10) * s, (m02 + m20) * s, (m21 - m12) * s};
        }
        if (m11 >= m22) {
            const T qy = T(0.5) * std::sqrt(T(1) - m00 + m11 - m22);
            const T s = T(0.25) / qy;
            return {(m01 + m10) * s, qy, (m12 + m21) * s, (m02 - m20) * s};
        }
        const T qz = T(0.5) * std::sqrt(T(1) - m00 - m11 + m22);
        const T s = T(0.25) / qz;
        return {(m02 + m20) * s, (m12 + m21) * s, qz, (m10 - m01) * s};
    }

    template <std::floating_point T>
    Quaternion<T> Quaternion<T>::lookRotation(const Vector<3, T>& _forward, const Vector<3, T>& _up) noexcept {
        const Vector<3, T> f = _forward.normalized();
        Vector<3, T> r = cross(f, _up);
        if (r.lengthSq() < T(1e-12))
            r = cross(f, std::abs(f.x) < T(0.9) ? Vector<3, T>{T(1), T(0), T(0)} : Vector<3, T>{T(0), T(0), T(1)});
        r.normalize();
        const Vector<3, T> u = cross(r, f);
        return fromBasis(r, u, -f);
    }

    template <std::floating_point T>
    Vector<3, T> Quaternion<T>::toEulerDeg(T _eps) const noexcept {
        const T r00 = T(1) - T(2) * (y * y + z * z);
        const T r01 = T(2) * (x * y - w * z);
        const T r02 = T(2) * (x * z + w * y);
        const T r10 = T(2) * (x * y + w * z);
        const T r20 = T(2) * (x * z - w * y);
        const T r21 = T(2) * (y * z + w * x);
        const T r22 = T(1) - T(2) * (x * x + y * y);

        T ax, ay, az;
        if (r20 <= T(-1) + _eps) {
            ay = constants::halfPi<T>;
            az = T(0);
            ax = std::atan2(r01, r02);
        } else if (r20 >= T(1) - _eps) {
            ay = -constants::halfPi<T>;
            az = T(0);
            ax = std::atan2(-r01, -r02);
        } else {
            ay = std::asin(-r20);
            ax = std::atan2(r21, r22);
            az = std::atan2(r10, r00);
        }
        return {ax * constants::radToDeg<T>, ay * constants::radToDeg<T>, az * constants::radToDeg<T>};
    }

    template <std::floating_point T>
    AxisAngle<T> Quaternion<T>::toAxisAngle(T _eps) const noexcept {
        const Quaternion q = (w < T(0) ? -*this : *this).normalized();
        const T s = std::sqrt(std::max(T(0), T(1) - q.w * q.w));
        if (s < _eps)
            return {{T(1), T(0), T(0)}, T(0)};
        return {{q.x / s, q.y / s, q.z / s}, T(2) * std::acos(std::min(T(1), q.w))};
    }

    template <std::floating_point T>
    constexpr Quaternion<T> Quaternion<T>::operator*(const Quaternion& _q) const noexcept {
        return {w * _q.x + x * _q.w + y * _q.z - z * _q.y, w * _q.y - x * _q.z + y * _q.w + z * _q.x, w * _q.z + x * _q.y - y * _q.x + z * _q.w, w * _q.w - x * _q.x - y * _q.y - z * _q.z};
    }

    template <std::floating_point T>
    constexpr Vector<3, T> Quaternion<T>::rotate(const Vector<3, T>& _v) const noexcept {
        const Vector<3, T> u{x, y, z};
        const Vector<3, T> t = T(2) * cross(u, _v);
        return _v + w * t + cross(u, t);
    }

} // namespace ee::math
