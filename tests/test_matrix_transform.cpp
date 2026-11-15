#include <doctest/doctest.h>

#include "math/Matrix.hpp"
#include "math/Quaternion.hpp"

using ee::math::Matrix;
using ee::math::Quaternion;
using Mat3 = Matrix<3, float>;
using Mat4 = Matrix<4, float>;
using Vec3 = ee::math::Vector3<float>;

namespace
{
    Vec3 applyPoint(const Mat4 &_m, const Vec3 &_p)
    {
        return {_m(0, 0) * _p.x + _m(0, 1) * _p.y + _m(0, 2) * _p.z + _m(0, 3),
                _m(1, 0) * _p.x + _m(1, 1) * _p.y + _m(1, 2) * _p.z + _m(1, 3),
                _m(2, 0) * _p.x + _m(2, 1) * _p.y + _m(2, 2) * _p.z + _m(2, 3)};
    }

    void checkVec(const Vec3 &_a, const Vec3 &_b, double _eps = 1e-4)
    {
        CHECK(_a.x == doctest::Approx(_b.x).epsilon(_eps));
        CHECK(_a.y == doctest::Approx(_b.y).epsilon(_eps));
        CHECK(_a.z == doctest::Approx(_b.z).epsilon(_eps));
    }

    const Quaternion kQuats[] = {
        Quaternion::identity(),
        Quaternion::fromEulerDeg(90.f, 0.f, 0.f),
        Quaternion::fromEulerDeg(0.f, 90.f, 0.f),
        Quaternion::fromEulerDeg(0.f, 0.f, 90.f),
        Quaternion::fromEulerDeg(30.f, 45.f, 60.f),
        Quaternion::fromEulerDeg(-120.f, 10.f, 170.f),
    };
}

TEST_CASE("translation et scaling : cases attendues")
{
    const Mat4 t = Mat4::translation({1.f, 2.f, 3.f});
    CHECK(t.m[12] == 1.f);
    CHECK(t.m[13] == 2.f);
    CHECK(t.m[14] == 3.f);
    CHECK(t.m[15] == 1.f);
    CHECK(Mat4::translation({0.f, 0.f, 0.f}) == Mat4::identity());

    const Mat4 s = Mat4::scaling({2.f, 3.f, 4.f});
    CHECK(s(0, 0) == 2.f);
    CHECK(s(1, 1) == 3.f);
    CHECK(s(2, 2) == 4.f);
    CHECK(s(3, 3) == 1.f);
    CHECK(Mat4::scaling({1.f, 1.f, 1.f}) == Mat4::identity());
}

TEST_CASE("translation : deplace un point, rien d'autre")
{
    const Mat4 t = Mat4::translation({1.f, 2.f, 3.f});
    checkVec(applyPoint(t, {0.f, 0.f, 0.f}), {1.f, 2.f, 3.f});
    checkVec(applyPoint(t, {5.f, -1.f, 2.f}), {6.f, 1.f, 5.f});
}

TEST_CASE("scaling : met un point a l'echelle sur chaque axe")
{
    const Mat4 s = Mat4::scaling({2.f, 3.f, 4.f});
    checkVec(applyPoint(s, {1.f, 1.f, 1.f}), {2.f, 3.f, 4.f});
    checkVec(applyPoint(s, {0.f, 0.f, 0.f}), {0.f, 0.f, 0.f});
}

TEST_CASE("rotation : colonne c == q.rotate(axe c)")
{
    const Vec3 axes[] = {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}};
    for (const Quaternion &q : kQuats)
    {
        CAPTURE(q.w); CAPTURE(q.x); CAPTURE(q.y); CAPTURE(q.z);
        const Mat4 r = Mat4::rotation(q);
        for (std::size_t c = 0; c < 3; ++c)
        {
            CAPTURE(c);
            checkVec({r(0, c), r(1, c), r(2, c)}, q.rotate(axes[c]));
        }
    }
}

TEST_CASE("rotation : un point tourne comme avec q.rotate")
{
    const Vec3 p{1.5f, -2.f, 0.75f};
    for (const Quaternion &q : kQuats)
        checkVec(applyPoint(Mat4::rotation(q), p), q.rotate(p));
}

TEST_CASE("rotation : 90 degres autour de Y envoie X sur -Z")
{
    const Mat4 r = Mat4::rotation(Quaternion::fromEulerDeg(0.f, 90.f, 0.f));
    checkVec(applyPoint(r, {1.f, 0.f, 0.f}), {0.f, 0.f, -1.f});
    checkVec(applyPoint(r, {0.f, 0.f, 1.f}), {1.f, 0.f, 0.f});
}

TEST_CASE("rotation : 4e ligne et 4e colonne = identite")
{
    const Mat4 r = Mat4::rotation(Quaternion::fromEulerDeg(30.f, 45.f, 60.f));
    for (std::size_t i = 0; i < 3; ++i)
    {
        CHECK(r(3, i) == 0.f);
        CHECK(r(i, 3) == 0.f);
    }
    CHECK(r(3, 3) == 1.f);
}

TEST_CASE("rotation : R * R^T == identite (pas de deformation)")
{
    for (const Quaternion &q : kQuats)
    {
        const Mat4 r = Mat4::rotation(q);
        CHECK((r * r.transposed()).nearlyEqual(Mat4::identity(), 1e-5f));
    }
}

TEST_CASE("rotation : quaternion non normalise = meme rotation")
{
    const Quaternion q = Quaternion::fromEulerDeg(30.f, 45.f, 60.f);
    const Quaternion big{q.w * 3.f, q.x * 3.f, q.y * 3.f, q.z * 3.f};
    CHECK(Mat4::rotation(big).nearlyEqual(Mat4::rotation(q), 1e-5f));
}

TEST_CASE("rotation : la 3x3 == le bloc 3x3 de la 4x4")
{
    for (const Quaternion &q : kQuats)
    {
        const Mat3 r3 = Mat3::rotation(q);
        const Mat4 r4 = Mat4::rotation(q);
        for (std::size_t row = 0; row < 3; ++row)
            for (std::size_t col = 0; col < 3; ++col)
                CHECK(r3(row, col) == doctest::Approx(r4(row, col)));
    }
}

TEST_CASE("rotation : Matrix<4, double>")
{
    using Mat4d = Matrix<4, double>;
    const Quaternion q = Quaternion::fromEulerDeg(30.f, 45.f, 60.f);
    const Mat4d rd = Mat4d::rotation(q);
    const Mat4 rf = Mat4::rotation(q);
    for (std::size_t i = 0; i < 16; ++i)
        CHECK(rd.m[i] == doctest::Approx(rf.m[i]).epsilon(1e-5));
}

TEST_CASE("fromTRS == translation * rotation * scaling")
{
    const Vec3 t{1.f, -2.f, 3.f};
    const Vec3 s{2.f, 3.f, 4.f};
    for (const Quaternion &q : kQuats)
    {
        const Mat4 ref = Mat4::translation(t) * Mat4::rotation(q) * Mat4::scaling(s);
        CHECK(Mat4::fromTRS(t, q, s).nearlyEqual(ref, 1e-4f));
    }
}

TEST_CASE("fromTRS : echelle, puis rotation, puis translation")
{
    const Quaternion q = Quaternion::fromEulerDeg(0.f, 90.f, 0.f);
    const Mat4 m = Mat4::fromTRS({10.f, 0.f, 0.f}, q, {2.f, 1.f, 1.f});
    checkVec(applyPoint(m, {1.f, 0.f, 0.f}), {10.f, 0.f, -2.f});
}
