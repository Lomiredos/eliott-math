#include <doctest/doctest.h>

#include <cmath>

#include "math/Matrix.hpp"

using ee::math::Matrix;
using Mat4 = Matrix<4, float>;
using Vec3 = ee::math::Vector3<float>;

namespace
{
    struct Vec4
    {
        float x, y, z, w;
    };

    Vec4 mul(const Mat4 &_m, const Vec4 &_v)
    {
        Vec4 r{};
        float *out[4] = {&r.x, &r.y, &r.z, &r.w};
        for (std::size_t row = 0; row < 4; ++row)
            *out[row] = _m(row, 0) * _v.x + _m(row, 1) * _v.y + _m(row, 2) * _v.z + _m(row, 3) * _v.w;
        return r;
    }

    Vec3 toNdc(const Mat4 &_m, const Vec3 &_p)
    {
        const Vec4 c = mul(_m, {_p.x, _p.y, _p.z, 1.f});
        return {c.x / c.w, c.y / c.w, c.z / c.w};
    }

    void checkVec(const Vec3 &_a, const Vec3 &_b, double _eps = 1e-4)
    {
        CHECK(_a.x == doctest::Approx(_b.x).epsilon(_eps));
        CHECK(_a.y == doctest::Approx(_b.y).epsilon(_eps));
        CHECK(_a.z == doctest::Approx(_b.z).epsilon(_eps));
    }

    void checkSame(const Mat4 &_mine, const float (&_ref)[16])
    {
        for (std::size_t i = 0; i < 16; ++i)
        {
            CAPTURE(i);
            CHECK(_mine.m[i] == doctest::Approx(_ref[i]).epsilon(1e-5));
        }
    }

    namespace ref
    {
        void perspective(float (&r)[16], float fovy, float aspect, float n, float f)
        {
            for (float &v : r) v = 0.f;
            const float t = std::tan(fovy * 0.5f);
            r[0] = 1.f / (aspect * t);
            r[5] = 1.f / t;
            r[10] = -(f + n) / (f - n);
            r[11] = -1.f;
            r[14] = -(2.f * f * n) / (f - n);
        }

        void ortho(float (&m)[16], float l, float r, float b, float t, float n, float f)
        {
            for (float &v : m) v = 0.f;
            m[0] = 2.f / (r - l);
            m[5] = 2.f / (t - b);
            m[10] = -2.f / (f - n);
            m[12] = -(r + l) / (r - l);
            m[13] = -(t + b) / (t - b);
            m[14] = -(f + n) / (f - n);
            m[15] = 1.f;
        }

        void normalize3(float v[3])
        {
            const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (l > 0.f) { v[0] /= l; v[1] /= l; v[2] /= l; }
        }

        void lookAt(float (&r)[16], const float eye[3], const float ctr[3], const float up[3])
        {
            float f[3] = {ctr[0] - eye[0], ctr[1] - eye[1], ctr[2] - eye[2]};
            normalize3(f);
            float s[3] = {f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0]};
            normalize3(s);
            const float u[3] = {s[1] * f[2] - s[2] * f[1], s[2] * f[0] - s[0] * f[2], s[0] * f[1] - s[1] * f[0]};
            for (float &v : r) v = 0.f;
            r[0] = s[0]; r[4] = s[1]; r[8] = s[2];
            r[1] = u[0]; r[5] = u[1]; r[9] = u[2];
            r[2] = -f[0]; r[6] = -f[1]; r[10] = -f[2];
            r[12] = -(s[0] * eye[0] + s[1] * eye[1] + s[2] * eye[2]);
            r[13] = -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]);
            r[14] = (f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2]);
            r[15] = 1.f;
        }
    }

    constexpr float kPi = 3.14159265358979323846f;
}

static_assert(Mat4::ortho(-1.f, 1.f, -1.f, 1.f, 1.f, -1.f) == Mat4::identity());
static_assert(Mat4::ortho(-2.f, 4.f, -1.f, 3.f, 0.5f, 50.f)(0, 3) == -(4.f + -2.f) / (4.f - -2.f));
static_assert(Mat4::ortho(-2.f, 4.f, -1.f, 3.f, 0.5f, 50.f)(3, 3) == 1.f);

TEST_CASE("perspective == celle de GLRenderer")
{
    const float args[][4] = {{1.0f, 1.6f, 0.1f, 100.f}, {kPi / 2.f, 1.f, 0.5f, 20.f}, {0.3f, 2.4f, 1.f, 1000.f}};
    for (const auto &a : args)
    {
        float r[16];
        ref::perspective(r, a[0], a[1], a[2], a[3]);
        checkSame(Mat4::perspective(a[0], a[1], a[2], a[3]), r);
    }
}

TEST_CASE("perspective : near -> z = -1, far -> z = +1")
{
    const float n = 0.1f, f = 100.f;
    const Mat4 p = Mat4::perspective(1.0f, 1.6f, n, f);
    CHECK(toNdc(p, {0.f, 0.f, -n}).z == doctest::Approx(-1.f));
    CHECK(toNdc(p, {0.f, 0.f, -f}).z == doctest::Approx(1.f).epsilon(1e-3));
}

TEST_CASE("perspective : w = -z (division perspective)")
{
    const Mat4 p = Mat4::perspective(1.0f, 1.6f, 0.1f, 100.f);
    CHECK(mul(p, {1.f, 2.f, -7.f, 1.f}).w == doctest::Approx(7.f));
}

TEST_CASE("perspective : le bord du champ de vision tombe sur +-1")
{
    const float fov = kPi / 3.f, aspect = 16.f / 9.f, d = 5.f;
    const Mat4 p = Mat4::perspective(fov, aspect, 0.1f, 100.f);
    const float h = d * std::tan(fov * 0.5f);
    CHECK(toNdc(p, {0.f, h, -d}).y == doctest::Approx(1.f));
    CHECK(toNdc(p, {0.f, -h, -d}).y == doctest::Approx(-1.f));
    CHECK(toNdc(p, {h * aspect, 0.f, -d}).x == doctest::Approx(1.f));
}

TEST_CASE("perspective : deux fois plus loin = deux fois plus petit")
{
    const Mat4 p = Mat4::perspective(1.0f, 1.f, 0.1f, 100.f);
    const float y1 = toNdc(p, {0.f, 1.f, -4.f}).y;
    const float y2 = toNdc(p, {0.f, 1.f, -8.f}).y;
    CHECK(y1 == doctest::Approx(2.f * y2));
}

TEST_CASE("ortho == celle de GLRenderer")
{
    float r[16];
    ref::ortho(r, -2.f, 4.f, -1.f, 3.f, 0.5f, 50.f);
    checkSame(Mat4::ortho(-2.f, 4.f, -1.f, 3.f, 0.5f, 50.f), r);
}

TEST_CASE("ortho : les coins de la boite vont sur les coins du cube [-1, 1]")
{
    const float l = -2.f, r = 4.f, b = -1.f, t = 3.f, n = 0.5f, f = 50.f;
    const Mat4 o = Mat4::ortho(l, r, b, t, n, f);
    checkVec(toNdc(o, {l, b, -n}), {-1.f, -1.f, -1.f});
    checkVec(toNdc(o, {r, t, -f}), {1.f, 1.f, 1.f});
    checkVec(toNdc(o, {(l + r) / 2.f, (b + t) / 2.f, -(n + f) / 2.f}), {0.f, 0.f, 0.f});
}

TEST_CASE("ortho : pas de perspective, w reste 1")
{
    const Mat4 o = Mat4::ortho(-2.f, 4.f, -1.f, 3.f, 0.5f, 50.f);
    CHECK(mul(o, {1.f, 2.f, -10.f, 1.f}).w == doctest::Approx(1.f));
    CHECK(toNdc(o, {0.f, 1.f, -4.f}).y == doctest::Approx(toNdc(o, {0.f, 1.f, -40.f}).y));
}

TEST_CASE("lookAt == celle de GLRenderer")
{
    const float cases[][9] = {
        {3.f, 2.f, 5.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f},
        {0.f, 0.f, 5.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f},
        {-4.f, 10.f, -2.f, 1.f, 0.f, 3.f, 0.f, 1.f, 0.f},
    };
    for (const auto &c : cases)
    {
        float r[16];
        ref::lookAt(r, c, c + 3, c + 6);
        checkSame(Mat4::lookAt({c[0], c[1], c[2]}, {c[3], c[4], c[5]}, {c[6], c[7], c[8]}), r);
    }
}

TEST_CASE("lookAt : la camera est a l'origine, la cible sur -Z")
{
    const Vec3 eye{3.f, 2.f, 5.f}, center{0.f, 1.f, 0.f};
    const Mat4 v = Mat4::lookAt(eye, center, {0.f, 1.f, 0.f});

    checkVec(toNdc(v, eye), {0.f, 0.f, 0.f});

    const Vec3 d = center - eye;
    const float dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    checkVec(toNdc(v, center), {0.f, 0.f, -dist});
}

TEST_CASE("lookAt : depuis +Z vers l'origine = translation de -5 en Z")
{
    const Mat4 v = Mat4::lookAt({0.f, 0.f, 5.f}, {0.f, 0.f, 0.f}, {0.f, 1.f, 0.f});
    CHECK(v.nearlyEqual(Mat4::translation({0.f, 0.f, -5.f})));
}

TEST_CASE("lookAt : la droite de la camera va sur +X, le haut sur +Y")
{
    const Mat4 v = Mat4::lookAt({0.f, 0.f, 5.f}, {0.f, 0.f, 0.f}, {0.f, 1.f, 0.f});
    checkVec(toNdc(v, {1.f, 0.f, 5.f}), {1.f, 0.f, 0.f});
    checkVec(toNdc(v, {0.f, 1.f, 5.f}), {0.f, 1.f, 0.f});
}

TEST_CASE("lookAt : rotation pure, R * R^T == identite")
{
    Mat4 v = Mat4::lookAt({-4.f, 10.f, -2.f}, {1.f, 0.f, 3.f}, {0.f, 1.f, 0.f});
    v(0, 3) = v(1, 3) = v(2, 3) = 0.f;
    CHECK((v * v.transposed()).nearlyEqual(Mat4::identity(), 1e-5f));
}

TEST_CASE("perspective * lookAt : un point devant la camera est visible")
{
    const Mat4 vp = Mat4::perspective(1.0f, 1.6f, 0.1f, 100.f) *
                    Mat4::lookAt({3.f, 2.f, 5.f}, {0.f, 1.f, 0.f}, {0.f, 1.f, 0.f});
    const Vec3 c = toNdc(vp, {0.f, 1.f, 0.f});
    CHECK(c.x == doctest::Approx(0.f).epsilon(1e-4));
    CHECK(c.y == doctest::Approx(0.f).epsilon(1e-4));
    CHECK(c.z > -1.f);
    CHECK(c.z < 1.f);
}

TEST_CASE("camera : Matrix<4, double> compile et donne la meme chose")
{
    using Mat4d = Matrix<4, double>;
    const Mat4d pd = Mat4d::perspective(1.0, 1.6, 0.1, 100.0);
    const Mat4 pf = Mat4::perspective(1.0f, 1.6f, 0.1f, 100.f);
    for (std::size_t i = 0; i < 16; ++i)
        CHECK(pd.m[i] == doctest::Approx(pf.m[i]).epsilon(1e-5));

    const Mat4d ld = Mat4d::lookAt({3.0, 2.0, 5.0}, {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0});
    CHECK(ld(3, 3) == 1.0);
}
