#include <doctest/doctest.h>

#include <array>
#include <bit>
#include <cstdint>
#include <type_traits>

#include "math/Vector.hpp"

using namespace ee::math;

using Vec2f = Vector<2, float>;
using Vec3f = Vector<3, float>;
using Vec4f = Vector<4, float>;
using Vec2i = Vector<2, int>;

namespace {
    template <class V>
    concept HasLength = requires(V _v) { _v.length(); };

    template <class V>
    concept HasNormalized = requires(V _v) { _v.normalized(); };

    template <class V>
    concept HasZ = requires(V _v) { _v.z; };

    template <class V>
    concept HasW = requires(V _v) { _v.w; };

    template <class V>
    concept HasCross = requires(V _v) { cross(_v, _v); };

    template <class V>
    concept HasXyz = requires(V _v) { _v.xyz(); };

    void checkVec(const Vec3f& _v, float _x, float _y, float _z) {
        CHECK(_v.x == doctest::Approx(_x));
        CHECK(_v.y == doctest::Approx(_y));
        CHECK(_v.z == doctest::Approx(_z));
    }
} // namespace

static_assert(sizeof(Vec2f) == 2 * sizeof(float));
static_assert(sizeof(Vec3f) == 3 * sizeof(float));
static_assert(sizeof(Vec4f) == 4 * sizeof(float));
static_assert(sizeof(Vector<3, double>) == 3 * sizeof(double));
static_assert(sizeof(Vector<4, std::uint8_t>) == 4);
static_assert(std::is_trivially_copyable_v<Vec4f>);
static_assert(std::is_standard_layout_v<Vec4f>);

static_assert(Vec3f{1, 2, 3}[0] == 1 && Vec3f{1, 2, 3}[1] == 2 && Vec3f{1, 2, 3}[2] == 3);
static_assert(Vec4f{1, 2, 3, 4}.w == 4);
static_assert(Vec3f{}.x == 0 && Vec3f{}.y == 0 && Vec3f{}.z == 0);
static_assert([] {
    Vec3f v{1, 2, 3};
    v[1] = 5;
    return v.y == 5;
}());

static_assert(Vec3f{1, 2, 3} + Vec3f{4, 5, 6} == Vec3f{5, 7, 9});
static_assert(Vec3f{4, 5, 6} - Vec3f{1, 2, 3} == Vec3f{3, 3, 3});
static_assert(Vec3f{1, 2, 3} * Vec3f{2, 3, 4} == Vec3f{2, 6, 12});
static_assert(Vec3f{1, 2, 3} * 2.f == Vec3f{2, 4, 6});
static_assert(2.f * Vec3f{1, 2, 3} == Vec3f{2, 4, 6});
static_assert(Vec3f{2, 4, 8} / 2.f == Vec3f{1, 2, 4});
static_assert(-Vec3f{1, -2, 3} == Vec3f{-1, 2, -3});
static_assert([] {
    Vec3f v{1, 2, 3};
    v += Vec3f{1, 1, 1};
    v *= 2.f;
    return v == Vec3f{4, 6, 8};
}());

static_assert(dot(Vec3f{1, 2, 3}, Vec3f{4, 5, 6}) == 32);
static_assert(cross(Vec3f{1, 0, 0}, Vec3f{0, 1, 0}) == Vec3f{0, 0, 1});
static_assert(cross(Vec2i{1, 0}, Vec2i{0, 1}) == 1);
static_assert(Vec4f{1, 2, 3, 4}.xyz() == Vec3f{1, 2, 3});

static_assert(HasLength<Vec3f> && !HasLength<Vec2i>);
static_assert(HasNormalized<Vec2f> && !HasNormalized<Vec2i>);
static_assert(HasZ<Vec3f> && !HasZ<Vec2f>);
static_assert(HasW<Vec4f> && !HasW<Vec3f>);
static_assert(HasCross<Vec3f> && HasCross<Vec2i> && !HasCross<Vec4f>);
static_assert(HasXyz<Vec4f> && !HasXyz<Vec3f>);

static_assert(std::same_as<Vec2i::WideType, std::int64_t>);
static_assert(std::same_as<Vector<2, unsigned>::WideType, std::uint64_t>);
static_assert(std::same_as<Vec2f::WideType, float>);
static_assert(std::same_as<decltype(dot(Vec2i{}, Vec2i{})), std::int64_t>);

static_assert(std::is_convertible_v<Vec2i, Vec2f>);
static_assert(!std::is_convertible_v<Vec2f, Vec2i>);
static_assert(std::is_constructible_v<Vec2i, Vec2f>);
static_assert(!std::is_convertible_v<double, Vec2i>);

TEST_CASE("Vector : length et lengthSq") {
    CHECK(Vec3f{3, 4, 0}.lengthSq() == 25);
    CHECK(Vec3f{3, 4, 0}.length() == doctest::Approx(5));
    CHECK(Vec2f{0, 0}.length() == 0);
    CHECK(Vec4f{1, 1, 1, 1}.length() == doctest::Approx(2));
}

TEST_CASE("Vector : normalized et normalize") {
    const Vec3f n = Vec3f{3, 4, 0}.normalized();
    checkVec(n, 0.6f, 0.8f, 0.f);
    CHECK(n.length() == doctest::Approx(1));

    Vec3f v{0, 0, 10};
    v.normalize();
    checkVec(v, 0, 0, 1);

    const Vec3f samples[] = {{1, 2, 3}, {-7, 0.5f, 100}, {1e-3f, 1e-3f, 0}, {1e4f, -1e4f, 1e4f}};
    for (const Vec3f& s : samples)
        CHECK(s.normalized().length() == doctest::Approx(1));
}

TEST_CASE("Vector : normalizedOr sur un vecteur nul") {
    int called = 0;
    const Vec3f zero = Vec3f{}.normalizedOr([&] { ++called; });
    CHECK(called == 1);
    CHECK(zero == Vec3f{});

    const Vec3f ok = Vec3f{0, 2, 0}.normalizedOr([&] { ++called; });
    CHECK(called == 1);
    checkVec(ok, 0, 1, 0);
}

TEST_CASE("Vector : distance et distanceSq") {
    const Vec3f a{1, 2, 3};
    const Vec3f b{4, 6, 3};
    CHECK(distance(a, b) == doctest::Approx(5));
    CHECK(distance(a, b) == doctest::Approx(distance(b, a)));
    CHECK(distanceSq(a, b) == doctest::Approx(25));
    CHECK(distance(a, a) == 0);
}

TEST_CASE("Vector : lerp et ease") {
    const Vec3f a{0, 10, -4};
    const Vec3f b{10, 20, 4};
    CHECK(lerp(a, b, 0.f) == a);
    CHECK(lerp(a, b, 1.f) == b);
    checkVec(lerp(a, b, 0.5f), 5, 15, 0);
    checkVec(lerp(a, b, 0.25f), 2.5f, 12.5f, -2);

    checkVec(ease(a, b, 0.5f, easing::smoothstep<float>), 5, 15, 0);
    checkVec(ease(a, b, 0.25f, easing::inQuad<float>), 0.625f, 10.625f, -3.5f);
    CHECK(ease(a, b, 1.f, [](float _t) { return _t * _t * _t; }) == b);
}

TEST_CASE("Vector : nearlyEqual") {
    const Vec3f a{1, 2, 3};
    CHECK(nearlyEqual(a, a));
    CHECK(nearlyEqual(a, Vec3f{1.000001f, 2.f, 2.999999f}));
    CHECK_FALSE(nearlyEqual(a, Vec3f{1.1f, 2, 3}));
    CHECK_FALSE(nearlyEqual(a, Vec3f{1, 2, 2.9f}));
    CHECK(nearlyEqual(a, Vec3f{1.05f, 2, 3}, 0.1f));

    CHECK_FALSE(nearlyEqual(Vec2f{0, 0}, Vec2f{100, 0}));
    CHECK_FALSE(nearlyEqual(Vec2f{1, 0}, Vec2f{0, 1}));

    const Vec3f n = Vec3f{1, 2, 3}.normalized();
    CHECK(nearlyEqual(n, n.normalized()));
}

TEST_CASE("Vector : cross, propriétés") {
    const Vec3f pairs[][2] = {
        {{1, 2, 3}, {4, 5, 6}},
        {{-1, 0.5f, 2}, {3, -2, 0.25f}},
        {{10, 0, 0}, {0, 0, -3}},
    };
    for (const auto& p : pairs) {
        const Vec3f& a = p[0];
        const Vec3f& b = p[1];
        const Vec3f c = cross(a, b);
        CHECK(dot(c, a) == doctest::Approx(0).epsilon(1e-5));
        CHECK(dot(c, b) == doctest::Approx(0).epsilon(1e-5));
        CHECK(nearlyEqual(cross(b, a), -c));
    }
    CHECK(cross(Vec3f{1, 2, 3}, Vec3f{2, 4, 6}) == Vec3f{});

    CHECK(cross(Vec2i{1, 0}, Vec2i{1, 1}) > 0);
    CHECK(cross(Vec2i{1, 0}, Vec2i{1, -1}) < 0);
    CHECK(cross(Vec2i{2, 2}, Vec2i{3, 3}) == 0);
    CHECK(cross(Vec2i{3, 1}, Vec2i{1, 4}) == -cross(Vec2i{1, 4}, Vec2i{3, 1}));
}

TEST_CASE("Vector<2, int> : opérations entières") {
    const Vec2i a{3, -4};
    const Vec2i b{1, 2};
    CHECK(a + b == Vec2i{4, -2});
    CHECK(a - b == Vec2i{2, -6});
    CHECK(a * 2 == Vec2i{6, -8});
    CHECK(dot(a, b) == -5);
    CHECK(a.lengthSq() == 25);
    CHECK(distanceSq(a, b) == 40);

    CHECK(Vec2i{7, -7} / 2 == Vec2i{3, -3});
    CHECK(Vec2i{7, 9} / Vec2i{2, 4} == Vec2i{3, 2});

    CHECK(a.manhattan() == 7);
    CHECK(a.chebyshev() == 4);
    CHECK(Vec2i{0, 0}.chebyshev() == 0);
}

TEST_CASE("Vector<2, int> : pas de dépassement dans lengthSq et dot") {
    const Vec2i big{50000, 50000};
    CHECK(big.lengthSq() == 5'000'000'000LL);
    CHECK(dot(big, big) == 5'000'000'000LL);

    const Vec2i edge{INT32_MAX, INT32_MAX};
    CHECK(edge.lengthSq() == 2 * std::int64_t(INT32_MAX) * INT32_MAX);
    CHECK(edge.manhattan() == 2 * std::int64_t(INT32_MAX));
}

TEST_CASE("Vector : conversions entre types") {
    const Vec2i g{3, 4};
    const Vec2f f = g;
    CHECK(f == Vec2f{3, 4});
    CHECK(f.length() == doctest::Approx(5));

    const auto back = static_cast<Vec2i>(Vec2f{1.9f, -2.7f});
    CHECK(back == Vec2i{1, -2});

    const Vec2i direct{1.7, 2.9};
    CHECK(direct == Vec2i{1, 2});
}

TEST_CASE("Vector : toArray et contiguïté mémoire") {
    const Vec3f v{1, 2, 3};
    const std::array<float, 3> arr = v.toArray();
    CHECK(arr[0] == 1);
    CHECK(arr[1] == 2);
    CHECK(arr[2] == 3);

    const std::array<Vec3f, 3> verts{Vec3f{1, 2, 3}, Vec3f{4, 5, 6}, Vec3f{7, 8, 9}};
    const auto flat = std::bit_cast<std::array<float, 9>>(verts);
    for (std::size_t i = 0; i < 9; ++i)
        CHECK(flat[i] == float(i + 1));

    CHECK(reinterpret_cast<const char*>(&verts[1]) - reinterpret_cast<const char*>(&verts[0]) == 3 * sizeof(float));
}
