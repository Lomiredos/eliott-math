#include <chrono>
#include <cstdio>
#include <random>

#include "math/Matrix.hpp"
#include "math/Quaternion.hpp"
#include "math/Vector.hpp"

using namespace ee::math;

using V3 = Vector<3, float>;
using V4 = Vector<4, float>;
using Q = Quaternion<float>;
using M4 = Matrix<4, float>;

namespace {
    constexpr int kPool = 1024;
    constexpr int kIter = 1'000'000;
    constexpr int kRuns = 7;

    V3 v3[kPool];
    V4 v4[kPool];
    Q qs[kPool];
    M4 ms[kPool];

    template <class T>
    inline void keep(const T& _value) {
#if defined(__GNUC__) || defined(__clang__)
        asm volatile("" : : "r"(&_value) : "memory");
#else
        static volatile const T* escape;
        escape = &_value;
#endif
    }

    constexpr int at(int _i, int _offset = 0) {
        return (_i + _offset) & (kPool - 1);
    }

    template <class F>
    void run(const char* _name, F _body) {
        double best = 1e300;
        for (int r = 0; r < kRuns; ++r) {
            const auto t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < kIter; ++i)
                keep(_body(i));
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            best = ms < best ? ms : best;
        }
        std::printf("  %-26s %8.2f ms   %6.2f ns/op\n", _name, best, best * 1e6 / kIter);
    }

    void section(const char* _title) {
        std::printf("\n%s\n", _title);
    }
} // namespace

int main() {
#if !defined(__OPTIMIZE__) && !defined(NDEBUG)
    std::printf("ATTENTION : compile sans optimisation, les temps ne sont pas representatifs.\n");
#endif
    std::printf("%d appels par ligne, meilleur temps sur %d passes\n", kIter, kRuns);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> pos(-10.f, 10.f), ang(-180.f, 180.f), sca(0.5f, 3.f);
    for (int i = 0; i < kPool; ++i) {
        v3[i] = {pos(rng), pos(rng), pos(rng)};
        v4[i] = {pos(rng), pos(rng), pos(rng), 1.f};
        qs[i] = Q::fromEulerDeg(ang(rng), ang(rng), ang(rng));
        ms[i] = M4::fromTRS({pos(rng), pos(rng), pos(rng)}, qs[i], {sca(rng), sca(rng), sca(rng)});
    }

    section("Vector<3, float>");
    run("a + b", [](int i) { return (v3[at(i)] + v3[at(i, 1)]); });
    run("a * s", [](int i) { return (v3[at(i)] * 1.5f); });
    run("dot", [](int i) { return dot(v3[at(i)], v3[at(i, 1)]); });
    run("cross", [](int i) { return cross(v3[at(i)], v3[at(i, 1)]); });
    run("lengthSq", [](int i) { return v3[at(i)].lengthSq(); });
    run("length", [](int i) { return v3[at(i)].length(); });
    run("normalized", [](int i) { return v3[at(i)].normalized(); });
    run("distance", [](int i) { return distance(v3[at(i)], v3[at(i, 1)]); });
    run("lerp", [](int i) { return lerp(v3[at(i)], v3[at(i, 1)], 0.3f); });
    run("nearlyEqual", [](int i) { return nearlyEqual(v3[at(i)], v3[at(i, 1)]); });

    section("Quaternion<float>");
    run("q * q", [](int i) { return (qs[at(i)] * qs[at(i, 1)]); });
    run("rotate(v)", [](int i) { return qs[at(i)].rotate(v3[at(i, 3)]); });
    run("forward", [](int i) { return qs[at(i)].forward(); });
    run("normalized", [](int i) { return qs[at(i)].normalized(); });
    run("inverse", [](int i) { return qs[at(i)].inverse(); });
    run("fromAxisAngle", [](int i) { return Q::fromAxisAngle(v3[at(i)], v3[at(i)].x); });
    run("fromEulerDeg", [](int i) {
        const V3& v = v3[at(i)];
        return Q::fromEulerDeg(v.x * 18.f, v.y * 18.f, v.z * 18.f);
    });
    run("toEulerDeg", [](int i) { return qs[at(i)].toEulerDeg(); });
    run("lookRotation", [](int i) { return Q::lookRotation(v3[at(i)]); });
    run("fromTo", [](int i) { return Q::fromTo(v3[at(i)], v3[at(i, 1)]); });
    run("nlerp", [](int i) { return nlerp(qs[at(i)], qs[at(i, 1)], 0.3f); });
    run("slerp", [](int i) { return slerp(qs[at(i)], qs[at(i, 1)], 0.3f); });
    run("angleBetween", [](int i) { return angleBetween(qs[at(i)], qs[at(i, 1)]); });

    section("Matrix<4, float>");
    run("M * M", [](int i) { return (ms[at(i)] * ms[at(i, 7)]); });
    run("M * Vector<4>", [](int i) { return (ms[at(i)] * v4[at(i, 3)]); });
    run("transformPoint", [](int i) { return ms[at(i)].transformPoint(v3[at(i, 3)]); });
    run("transformDir", [](int i) { return ms[at(i)].transformDir(v3[at(i, 3)]); });
    run("projectPoint", [](int i) { return ms[at(i)].projectPoint(v3[at(i, 3)]); });
    run("transposed", [](int i) { return ms[at(i)].transposed(); });
    run("determinant", [](int i) { return ms[at(i)].determinant(); });
    run("inverse", [](int i) { return *ms[at(i)].inverse(); });
    run("inverseAffine", [](int i) { return ms[at(i)].inverseAffine(); });
    run("normalMatrix", [](int i) { return ms[at(i)].normalMatrix(); });
    run("fromTRS", [](int i) { return M4::fromTRS(v3[at(i)], qs[at(i)], v3[at(i, 1)]); });
    run("rotation(q)", [](int i) { return M4::rotation(qs[at(i)]); });
    run("lookAt", [](int i) { return M4::lookAt(v3[at(i)], v3[at(i, 1)], {0.f, 1.f, 0.f}); });
    run("decompose", [](int i) {
        V3 t, s;
        Q r;
        keep(ms[at(i)].decompose(t, r, s));
        keep(t);
        keep(s);
        return r;
    });
}
