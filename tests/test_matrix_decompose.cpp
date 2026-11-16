#include <doctest/doctest.h>

#include <cmath>

#include "math/Matrix.hpp"
#include "math/Quaternion.hpp"

using ee::math::Matrix;
using Quaternion = ee::math::Quaternion<float>;
using Mat3 = Matrix<3, float>;
using Mat4 = Matrix<4, float>;
using Vec3 = ee::math::Vector<3, float>;

namespace
{
    constexpr float kEps = 1e-4f;

    bool near(float _a, float _b, float _eps = kEps) { return std::fabs(_a - _b) <= _eps; }

    bool nearVec(const Vec3 &_a, const Vec3 &_b, float _eps = kEps)
    {
        return near(_a.x, _b.x, _eps) && near(_a.y, _b.y, _eps) && near(_a.z, _b.z, _eps);
    }

    struct TRS
    {
        const char *name;
        Vec3 t;
        Quaternion r;
        Vec3 s;
    };

    const TRS kPositive[] = {
        {"identite", {0, 0, 0}, Quaternion::identity(), {1, 1, 1}},
        {"translation seule", {1, -2, 3}, Quaternion::identity(), {1, 1, 1}},
        {"rotation 30 x", {0, 0, 0}, Quaternion::fromEulerDeg(30, 0, 0), {1, 1, 1}},
        {"rotation quelconque", {4, 5, -6}, Quaternion::fromEulerDeg(25, -70, 140), {1, 1, 1}},
        {"echelle non uniforme", {0, 0, 0}, Quaternion::identity(), {2, 3, 4}},
        {"TRS complet", {1, 2, 3}, Quaternion::fromEulerDeg(10, 20, 30), {2, 3, 4}},
        {"180 x + echelle", {-1, 0, 2}, Quaternion::fromEulerDeg(180, 0, 0), {0.5f, 1, 2}},
        {"170 autour de (1,1,0)", {0, 3, 0},
         Quaternion::fromAxisAngle({0.70710678f, 0.70710678f, 0}, 170.0f * 3.14159265f / 180.0f), {1, 2, 1}},
    };
}

TEST_CASE("toQuaternion : rotation(q).toQuaternion() == +-q, dans les 4 branches")
{
    const float k = 3.14159265f / 180.0f;
    const Quaternion cases[] = {
        Quaternion::identity(),                       // branche w
        Quaternion::fromAxisAngle({1, 0, 0}, 180 * k), // branche x
        Quaternion::fromAxisAngle({0, 1, 0}, 180 * k), // branche y
        Quaternion::fromAxisAngle({0, 0, 1}, 180 * k), // branche z
        Quaternion::fromAxisAngle({1, 0, 0}, 160 * k),
        Quaternion::fromAxisAngle({0, 1, 0}, 160 * k),
        Quaternion::fromAxisAngle({0, 0, 1}, 160 * k),
        Quaternion::fromEulerDeg(30, 0, 0),
        Quaternion::fromEulerDeg(25, -70, 140),
        Quaternion::fromEulerDeg(-120, 45, 200),
    };
    for (const Quaternion &q : cases)
    {
        CAPTURE(q.w);
        CAPTURE(q.x);
        CAPTURE(q.y);
        CAPTURE(q.z);
        const Quaternion r = Mat3::rotation(q).toQuaternion();
        CHECK(sameRotation(q, r, kEps));
        CHECK(near(dot(r, r), 1.0f));
        CHECK(Mat3::rotation(r).nearlyEqual(Mat3::rotation(q), kEps));
    }
}

TEST_CASE("toQuaternion : branche x avec des composantes non nulles (signes)")
{
    const Quaternion q = Quaternion::fromAxisAngle({0.6f, 0.0f, 0.8f}, 3.0f);
    const Quaternion r = Mat3::rotation(q).toQuaternion();
    CHECK(sameRotation(q, r, kEps));
}

TEST_CASE("decompose : retrouve t, s et la rotation (echelles positives)")
{
    for (const TRS &c : kPositive)
    {
        CAPTURE(c.name);
        const Mat4 m = Mat4::fromTRS(c.t, c.r, c.s);
        Vec3 t, s;
        Quaternion r;
        REQUIRE(m.decompose(t, r, s));
        CHECK(nearVec(t, c.t));
        CHECK(nearVec(s, c.s));
        CHECK(sameRotation(r, c.r, kEps));
        CHECK(Mat4::fromTRS(t, r, s).nearlyEqual(m, kEps));
    }
}

TEST_CASE("decompose : echelle negative -> meme matrice recomposee, rotation valide")
{
    const TRS cases[] = {
        {"miroir x seul", {0, 0, 0}, Quaternion::identity(), {-1, 1, 1}},
        {"miroir y seul", {0, 0, 0}, Quaternion::identity(), {1, -1, 1}},
        {"rotation z + miroir x", {0, 0, 0}, Quaternion::fromEulerDeg(0, 0, 30), {-1, 1, 1}},
        {"TRS + echelle {-1, 2, 0.5}", {1, 2, 3}, Quaternion::fromEulerDeg(10, 20, 30), {-1, 2, 0.5f}},
        {"trois miroirs", {0, 1, 0}, Quaternion::fromEulerDeg(45, 0, 0), {-2, -3, -4}},
    };
    for (const TRS &c : cases)
    {
        CAPTURE(c.name);
        const Mat4 m = Mat4::fromTRS(c.t, c.r, c.s);
        Vec3 t, s;
        Quaternion r;
        REQUIRE(m.decompose(t, r, s));
        CHECK(nearVec(t, c.t));
        CHECK(near(std::fabs(s.x * s.y * s.z), std::fabs(c.s.x * c.s.y * c.s.z)));
        CHECK(s.x * s.y * s.z < 0.0f);
        CHECK(near(Mat3::rotation(r).determinant(), 1.0f));
        CHECK(Mat4::fromTRS(t, r, s).nearlyEqual(m, kEps));
    }
}

TEST_CASE("decompose : echelle nulle -> false")
{
    Vec3 t, s;
    Quaternion r;
    CHECK_FALSE(Mat4::fromTRS({1, 2, 3}, Quaternion::identity(), {0, 1, 1}).decompose(t, r, s));
    CHECK_FALSE(Mat4::fromTRS({1, 2, 3}, Quaternion::fromEulerDeg(10, 20, 30), {1, 1, 0}).decompose(t, r, s));
    CHECK_FALSE(Mat4{}.decompose(t, r, s));
}

TEST_CASE("decompose : lookAt est rigide (echelle 1)")
{
    const Mat4 v = Mat4::lookAt({3, 4, 5}, {0, 0, 0}, {0, 1, 0});
    Vec3 t, s;
    Quaternion r;
    REQUIRE(v.decompose(t, r, s));
    CHECK(nearVec(s, {1, 1, 1}));
    CHECK(Mat4::fromTRS(t, r, s).nearlyEqual(v, kEps));
}

TEST_CASE("decompose puis recompose sur un produit de matrices")
{
    const Mat4 parent = Mat4::fromTRS({0, 5, 0}, Quaternion::fromEulerDeg(0, 90, 0), {2, 2, 2});
    const Mat4 child = Mat4::fromTRS({1, 0, 0}, Quaternion::fromEulerDeg(30, 0, 0), {1, 1, 1});
    const Mat4 world = parent * child;
    Vec3 t, s;
    Quaternion r;
    REQUIRE(world.decompose(t, r, s));
    CHECK(nearVec(t, world.getTranslation()));
    CHECK(nearVec(s, {2, 2, 2}));
    CHECK(Mat4::fromTRS(t, r, s).nearlyEqual(world, kEps));
}
