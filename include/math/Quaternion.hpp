#pragma once
#include <cmath>
#include "Vector3.hpp"

namespace ee::math {

struct Quaternion {
    float w = 1.f, x = 0.f, y = 0.f, z = 0.f;

    static Quaternion identity() { return { 1.f, 0.f, 0.f, 0.f }; }


    static Quaternion fromAxisAngle(Vector3<float> axis, float angleRad) {
        float half = angleRad * 0.5f;
        float s    = sinf(half);
        return { cosf(half), axis.x * s, axis.y * s, axis.z * s };
    }

    Quaternion operator*(const Quaternion& q) const {
        return {
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        };
    }

    Quaternion Normalize() const {
        float len = sqrtf(w*w + x*x + y*y + z*z);
        if (len > 0.f) return { w/len, x/len, y/len, z/len };
        return identity();
    }

    Vector3<float> rotate(Vector3<float> v) const {
        Quaternion qv  = { 0.f, v.x, v.y, v.z };
        Quaternion conj = { w, -x, -y, -z };
        Quaternion res = (*this * qv) * conj;
        return { res.x, res.y, res.z };
    }

    Vector3<float> forward() const { return rotate({ 0,  0, -1 }); }
    Vector3<float> back()    const { return rotate({ 0,  0,  1 }); }
    Vector3<float> up()      const { return rotate({ 0,  1,  0 }); }
    Vector3<float> right()   const { return rotate({ 1,  0,  0 }); }
};

}
