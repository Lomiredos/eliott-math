#include <doctest/doctest.h>

#include <cmath>

#include "math/Matrix.hpp"
#include "math/Quaternion.hpp"

using ee::math::Matrix;
using ee::math::Quaternion;
using Mat3 = Matrix<3, float>;
using Mat4 = Matrix<4, float>;
using Vec3 = ee::math::Vector3<float>;

namespace
{
    void checkVec(const Vec3 &_a, const Vec3 &_b, double _eps = 1e-4)
    {
        CHECK(_a.x == doctest::Approx(_b.x).epsilon(_eps));
        CHECK(_a.y == doctest::Approx(_b.y).epsilon(_eps));
        CHECK(_a.z == doctest::Approx(_b.z).epsilon(_eps));
    }

    float dot(const Vec3 &_a, const Vec3 &_b) { return _a.x * _b.x + _a.y * _b.y + _a.z * _b.z; }

    Vec3 sub(const Vec3 &_a, const Vec3 &_b) { return {_a.x - _b.x, _a.y - _b.y, _a.z - _b.z}; }

    Vec3 refMul4(const Mat4 &_m, const Vec3 &_v, float _w)
    {
        const float b[4]{_v.x, _v.y, _v.z, _w};
        float r[4]{};
        for (std::size_t row = 0; row < 4; ++row)
            for (std::size_t k = 0; k < 4; ++k)
                r[row] += _m(row, k) * b[k];
        return {r[0], r[1], r[2]};
    }

    const Quaternion kQ = Quaternion::fromEulerDeg(30.f, 45.f, 60.f);
    const Vec3 kT{1.f, -2.f, 3.f};
    const Vec3 kS{2.f, 3.f, 4.f};
}

TEST_CASE("getTranslation : la 4e colonne")
{
    checkVec(Mat4::translation(kT).getTranslation(), kT);
    checkVec(Mat4::fromTRS(kT, kQ, kS).getTranslation(), kT);
    checkVec(Mat4::identity().getTranslation(), {0.f, 0.f, 0.f});
}

TEST_CASE("transformPoint : l'origine arrive sur la translation")
{
    checkVec(Mat4::translation(kT).transformPoint({0.f, 0.f, 0.f}), kT);
    checkVec(Mat4::fromTRS(kT, kQ, kS).transformPoint({0.f, 0.f, 0.f}), kT);
}

TEST_CASE("transformPoint == produit 4x4 complet avec w = 1")
{
    const Mat4 m = Mat4::fromTRS(kT, kQ, kS);
    const Vec3 pts[] = {{1.f, 0.f, 0.f}, {-3.f, 2.f, 0.5f}, {10.f, -7.f, 4.f}};
    for (const Vec3 &p : pts)
        checkVec(m.transformPoint(p), refMul4(m, p, 1.f));
}

TEST_CASE("transformPoint : echelle, puis rotation, puis translation")
{
    const Mat4 m = Mat4::fromTRS({10.f, 0.f, 0.f}, Quaternion::fromEulerDeg(0.f, 90.f, 0.f), {2.f, 1.f, 1.f});
    checkVec(m.transformPoint({1.f, 0.f, 0.f}), {10.f, 0.f, -2.f});
}

TEST_CASE("transformDir : ignore la translation")
{
    const Mat4 t = Mat4::translation(kT);
    checkVec(t.transformDir({1.f, 2.f, 3.f}), {1.f, 2.f, 3.f});
    checkVec(t.transformDir({0.f, 0.f, 0.f}), {0.f, 0.f, 0.f});
}

TEST_CASE("transformDir == produit 4x4 complet avec w = 0")
{
    const Mat4 m = Mat4::fromTRS(kT, kQ, kS);
    const Vec3 dirs[] = {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {-3.f, 2.f, 0.5f}};
    for (const Vec3 &d : dirs)
        checkVec(m.transformDir(d), refMul4(m, d, 0.f));
}

TEST_CASE("transformDir : rotation pure == q.rotate")
{
    const Mat4 r = Mat4::rotation(kQ);
    const Vec3 v{1.5f, -2.f, 0.75f};
    checkVec(r.transformDir(v), kQ.rotate(v));
}

TEST_CASE("point et direction : P(b) - P(a) == D(b - a)")
{
    const Mat4 m = Mat4::fromTRS(kT, kQ, kS);
    const Vec3 a{1.f, 2.f, 3.f}, b{-4.f, 0.5f, 2.f};
    checkVec(sub(m.transformPoint(b), m.transformPoint(a)), m.transformDir(sub(b, a)));
}

TEST_CASE("projectPoint : identique a transformPoint pour une matrice affine")
{
    const Mat4 m = Mat4::fromTRS(kT, kQ, kS);
    const Vec3 p{-3.f, 2.f, 0.5f};
    checkVec(m.projectPoint(p), m.transformPoint(p));
}

TEST_CASE("projectPoint : perspective, near -> -1, far -> +1")
{
    const Mat4 p = Mat4::perspective(1.0f, 1.6f, 0.1f, 100.f);
    CHECK(p.projectPoint({0.f, 0.f, -0.1f}).z == doctest::Approx(-1.f));
    CHECK(p.projectPoint({0.f, 0.f, -100.f}).z == doctest::Approx(1.f).epsilon(1e-3));
}

TEST_CASE("projectPoint : w = 0 ne donne pas d'infini")
{
    const Mat4 p = Mat4::perspective(1.0f, 1.6f, 0.1f, 100.f);
    const Vec3 r = p.projectPoint({1.f, 1.f, 0.f});
    CHECK(std::isfinite(r.x));
    CHECK(std::isfinite(r.y));
    CHECK(std::isfinite(r.z));
}

TEST_CASE("upperLeft3 : le bloc 3x3, sans la translation")
{
    const Mat4 m = Mat4::fromTRS(kT, kQ, kS);
    const Mat3 ul = m.upperLeft3();
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t col = 0; col < 3; ++col)
            CHECK(ul(row, col) == m(row, col));

    CHECK(Mat4::translation(kT).upperLeft3() == Mat3::identity());
}

TEST_CASE("upperLeft3 d'une rotation == rotation 3x3")
{
    CHECK(Mat4::rotation(kQ).upperLeft3().nearlyEqual(Mat3::rotation(kQ)));
}

TEST_CASE("Matrix<3> * Vector3")
{
    const Vec3 v{1.5f, -2.f, 0.75f};
    checkVec(Mat3::identity() * v, v);
    checkVec(Mat3::rotation(kQ) * v, kQ.rotate(v));

    Mat3 a{};
    a(0, 0) = 1.f; a(0, 1) = 2.f; a(0, 2) = 3.f;
    a(1, 0) = 4.f; a(1, 1) = 5.f; a(1, 2) = 6.f;
    a(2, 0) = 7.f; a(2, 1) = 8.f; a(2, 2) = 9.f;
    checkVec(a * Vec3{1.f, 0.f, 0.f}, {1.f, 4.f, 7.f});
    checkVec(a * Vec3{1.f, 1.f, 1.f}, {6.f, 15.f, 24.f});
}

TEST_CASE("upperLeft3() * d == transformDir(d)")
{
    const Mat4 m = Mat4::fromTRS(kT, kQ, kS);
    const Vec3 d{-3.f, 2.f, 0.5f};
    checkVec(m.upperLeft3() * d, m.transformDir(d));
}

TEST_CASE("normalMatrix : echelle (1, 2, 1), la normale reste perpendiculaire")
{
    const Mat4 m = Mat4::scaling({1.f, 2.f, 1.f});
    const Vec3 tangent{1.f, -1.f, 0.f};
    const Vec3 normal{1.f, 1.f, 0.f};

    const Vec3 t2 = m.transformDir(tangent);
    CHECK(dot(m.normalMatrix() * normal, t2) == doctest::Approx(0.f));
    CHECK(dot(m.transformDir(normal), t2) != doctest::Approx(0.f));
}

TEST_CASE("normalMatrix : rotation + echelle non uniforme, normales perpendiculaires")
{
    const Mat4 m = Mat4::fromTRS(kT, kQ, kS);
    const Mat3 nm = m.normalMatrix();

    const Vec3 normal{1.f, 1.f, 0.f};
    const Vec3 tangents[] = {{1.f, -1.f, 0.f}, {0.f, 0.f, 1.f}, {1.f, -1.f, 2.f}};
    for (const Vec3 &t : tangents)
    {
        CAPTURE(t.x); CAPTURE(t.y); CAPTURE(t.z);
        REQUIRE(dot(normal, t) == doctest::Approx(0.f));
        CHECK(dot(nm * normal, m.transformDir(t)) == doctest::Approx(0.f).epsilon(1e-4));
    }
}

TEST_CASE("normalMatrix : C^T * A == det * I (inverse transposee a un facteur pres)")
{
    const Mat4 m = Mat4::fromTRS(kT, kQ, kS);
    const Mat3 a = m.upperLeft3();
    const float det = kS.x * kS.y * kS.z;
    CHECK((m.normalMatrix().transposed() * a).nearlyEqual(Mat3::identity() * det, 1e-3f));
}

TEST_CASE("normalMatrix : rotation pure == la rotation elle-meme")
{
    const Mat4 r = Mat4::rotation(kQ);
    CHECK(r.normalMatrix().nearlyEqual(r.upperLeft3()));
}

TEST_CASE("normalMatrix : ignore la translation")
{
    const Mat4 a = Mat4::fromTRS({0.f, 0.f, 0.f}, kQ, kS);
    const Mat4 b = Mat4::fromTRS({100.f, -50.f, 7.f}, kQ, kS);
    CHECK(a.normalMatrix() == b.normalMatrix());
}

TEST_CASE("normalMatrix : echelle uniforme == rotation (a un facteur pres)")
{
    const Mat3 nm = Mat4::fromTRS(kT, kQ, {3.f, 3.f, 3.f}).normalMatrix();
    CHECK(nm.nearlyEqual(Mat3::rotation(kQ) * 9.f, 1e-3f));
}
