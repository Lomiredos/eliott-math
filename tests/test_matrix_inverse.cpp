#include <doctest/doctest.h>

#include <cmath>

#include "math/Matrix.hpp"
#include "math/Quaternion.hpp"

using ee::math::Matrix;
using ee::math::Quaternion;
using Mat2 = Matrix<2, float>;
using Mat3 = Matrix<3, float>;
using Mat4 = Matrix<4, float>;
using Vec3 = ee::math::Vector3<float>;

namespace
{
    template <std::size_t N>
    Matrix<N, float> fromRows(const float (&_v)[N * N])
    {
        Matrix<N, float> r{};
        for (std::size_t row = 0; row < N; ++row)
            for (std::size_t col = 0; col < N; ++col)
                r(row, col) = _v[row * N + col];
        return r;
    }

    double laplace(const double *_x, int _n)
    {
        if (_n == 1) return _x[0];
        double s = 0.0;
        for (int j = 0; j < _n; ++j)
        {
            double sub[16];
            int k = 0;
            for (int r = 1; r < _n; ++r)
                for (int c = 0; c < _n; ++c)
                    if (c != j) sub[k++] = _x[r * _n + c];
            s += (j % 2 ? -1.0 : 1.0) * _x[j] * laplace(sub, _n - 1);
        }
        return s;
    }

    template <std::size_t N>
    double refDet(const Matrix<N, float> &_m)
    {
        double x[16];
        for (std::size_t row = 0; row < N; ++row)
            for (std::size_t col = 0; col < N; ++col)
                x[row * N + col] = _m(row, col);
        return laplace(x, static_cast<int>(N));
    }

    template <std::size_t N>
    void checkInverse(const Matrix<N, float> &_m, float _eps = 1e-4f)
    {
        const auto inv = _m.inverse();
        REQUIRE(inv.has_value());
        CHECK((_m * *inv).nearlyEqual(Matrix<N, float>::identity(), _eps));
        CHECK((*inv * _m).nearlyEqual(Matrix<N, float>::identity(), _eps));
    }

    const Quaternion kQ = Quaternion::fromEulerDeg(30.f, 45.f, 60.f);

    const Mat4 kAffine[] = {
        Mat4::identity(),
        Mat4::translation({-4.f, 7.f, 1.f}),
        Mat4::rotation(kQ),
        Mat4::scaling({2.f, 3.f, 4.f}),
        Mat4::fromTRS({1.f, 2.f, 3.f}, kQ, {2.f, 3.f, 4.f}),
        Mat4::fromTRS({0.f, 0.f, 0.f}, Quaternion::identity(), {-1.f, 2.f, 0.5f}),
        Mat4::lookAt({3.f, 2.f, 5.f}, {0.f, 1.f, 0.f}, {0.f, 1.f, 0.f}),
    };

    const float kGeneric4[] = {2, 1, 1, 0, 4, 3, 3, 1, 8, 7, 9, 2, 1, 0, 5, 3};
}

static_assert(Mat2::identity().determinant() == 1.f);
static_assert(Mat3::identity().determinant() == 1.f);
static_assert(Mat4::identity().determinant() == 1.f);
static_assert(Mat4::identity().inverse().has_value());
static_assert(*Mat4::identity().inverse() == Mat4::identity());

TEST_CASE("determinant : 2x2 et 3x3 calcules a la main")
{
    CHECK(fromRows<2>({2, 1, 4, 3}).determinant() == doctest::Approx(2.f));
    CHECK(fromRows<2>({-2, 1, 4, 3}).determinant() == doctest::Approx(-10.f));
    CHECK(fromRows<3>({2, 1, 1, 4, 3, 3, 8, 7, 9}).determinant() == doctest::Approx(4.f));
}

TEST_CASE("determinant == developpement de Laplace (2x2, 3x3, 4x4)")
{
    const Mat3 m3 = Mat4::fromTRS({1.f, 2.f, 3.f}, kQ, {2.f, 3.f, 4.f}).upperLeft3();
    CHECK(m3.determinant() == doctest::Approx(refDet(m3)).epsilon(1e-4));

    const Mat4 cases[] = {
        fromRows<4>(kGeneric4),
        fromRows<4>({0, 1, 2, 3, 1, 0, 4, 5, 2, 4, 0, 6, 3, 5, 6, 0}),
        Mat4::fromTRS({1.f, 2.f, 3.f}, kQ, {2.f, 3.f, 4.f}),
        Mat4::perspective(1.0f, 1.6f, 0.1f, 100.f),
    };
    for (const Mat4 &m : cases)
        CHECK(m.determinant() == doctest::Approx(refDet(m)).epsilon(1e-4));
}

TEST_CASE("determinant : matrices singulieres -> 0")
{
    CHECK(fromRows<2>({1, 2, 2, 4}).determinant() == doctest::Approx(0.f));
    CHECK(fromRows<3>({1, 2, 3, 4, 5, 6, 7, 8, 9}).determinant() == doctest::Approx(0.f));
    CHECK(fromRows<4>({1, 2, 3, 4, 2, 4, 6, 8, 0, 1, 0, 1, 5, 5, 5, 5}).determinant() == doctest::Approx(0.f));
}

TEST_CASE("determinant : echange de deux lignes = changement de signe")
{
    const Mat4 a = fromRows<4>(kGeneric4);
    Mat4 b = a;
    for (std::size_t col = 0; col < 4; ++col)
        std::swap(b(0, col), b(2, col));
    CHECK(b.determinant() == doctest::Approx(-a.determinant()));
}

TEST_CASE("determinant : det(A * B) == det(A) * det(B)")
{
    const Mat4 a = fromRows<4>(kGeneric4);
    const Mat4 b = Mat4::fromTRS({1.f, 2.f, 3.f}, kQ, {2.f, 3.f, 4.f});
    CHECK((a * b).determinant() == doctest::Approx(a.determinant() * b.determinant()).epsilon(1e-4));
}

TEST_CASE("determinant : scaling(s) = sx * sy * sz, rotation = 1, transposee = meme det")
{
    CHECK(Mat4::scaling({2.f, 3.f, 4.f}).determinant() == doctest::Approx(24.f));
    CHECK(Mat4::scaling({-1.f, 1.f, 1.f}).determinant() == doctest::Approx(-1.f));
    CHECK(Mat4::rotation(kQ).determinant() == doctest::Approx(1.f));
    CHECK(Mat3::rotation(kQ).determinant() == doctest::Approx(1.f));

    const Mat4 a = fromRows<4>(kGeneric4);
    CHECK(a.transposed().determinant() == doctest::Approx(a.determinant()));
}

TEST_CASE("inverse 2x2 calculee a la main")
{
    const auto inv = fromRows<2>({2, 1, 4, 3}).inverse();
    REQUIRE(inv.has_value());
    CHECK(inv->nearlyEqual(fromRows<2>({1.5f, -0.5f, -2.f, 1.f})));
}

TEST_CASE("inverse : A * A^-1 == A^-1 * A == I")
{
    checkInverse(fromRows<2>({2, 1, 4, 3}));
    checkInverse(fromRows<2>({-2, 1, 4, 3}));
    checkInverse(fromRows<3>({2, 1, 1, 4, 3, 3, 8, 7, 9}));
    checkInverse(Mat4::fromTRS({1.f, 2.f, 3.f}, kQ, {2.f, 3.f, 4.f}).upperLeft3());
    checkInverse(fromRows<4>(kGeneric4));
    checkInverse(Mat4::perspective(1.0f, 1.6f, 0.1f, 100.f), 1e-3f);
    for (const Mat4 &m : kAffine)
        checkInverse(m);
}

TEST_CASE("inverse : pivot nul au depart (echange de lignes)")
{
    checkInverse(fromRows<2>({0, 1, 1, 1}));
    checkInverse(fromRows<3>({0, 1, 2, 1, 0, 3, 4, -3, 8}));
    checkInverse(fromRows<4>({0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));
}

TEST_CASE("inverse : matrices singulieres -> nullopt")
{
    CHECK_FALSE(fromRows<2>({1, 2, 2, 4}).inverse().has_value());
    CHECK_FALSE(fromRows<3>({1, 2, 3, 4, 5, 6, 7, 8, 9}).inverse().has_value());
    CHECK_FALSE(fromRows<4>({1, 2, 3, 4, 2, 4, 6, 8, 0, 1, 0, 1, 5, 5, 5, 5}).inverse().has_value());
    CHECK_FALSE(Mat4::scaling({1.f, 0.f, 1.f}).inverse().has_value());
}

TEST_CASE("inverse : l'inverse de l'inverse redonne la matrice")
{
    const Mat4 a = fromRows<4>(kGeneric4);
    const auto inv = a.inverse();
    REQUIRE(inv.has_value());
    const auto back = inv->inverse();
    REQUIRE(back.has_value());
    CHECK(back->nearlyEqual(a, 1e-3f));
}

TEST_CASE("inverse : (A * B)^-1 == B^-1 * A^-1")
{
    const Mat4 a = fromRows<4>(kGeneric4);
    const Mat4 b = Mat4::fromTRS({1.f, 2.f, 3.f}, kQ, {2.f, 3.f, 4.f});
    CHECK((a * b).inverse()->nearlyEqual(*b.inverse() * *a.inverse(), 1e-3f));
}

TEST_CASE("inverse : Matrix<4, double>")
{
    using Mat4d = Matrix<4, double>;
    Mat4d a{};
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t col = 0; col < 4; ++col)
            a(row, col) = kGeneric4[row * 4 + col];
    const auto inv = a.inverse();
    REQUIRE(inv.has_value());
    CHECK((a * *inv).nearlyEqual(Mat4d::identity(), 1e-10));
}

TEST_CASE("inverseAffine == inverse() sur les matrices affines")
{
    for (const Mat4 &m : kAffine)
    {
        CHECK((m * m.inverseAffine()).nearlyEqual(Mat4::identity(), 1e-4f));
        CHECK(m.inverseAffine().nearlyEqual(*m.inverse(), 1e-4f));
    }
}

TEST_CASE("inverseAffine : derniere ligne (0, 0, 0, 1)")
{
    const Mat4 ia = Mat4::fromTRS({1.f, 2.f, 3.f}, kQ, {2.f, 3.f, 4.f}).inverseAffine();
    CHECK(ia(3, 0) == 0.f);
    CHECK(ia(3, 1) == 0.f);
    CHECK(ia(3, 2) == 0.f);
    CHECK(ia(3, 3) == 1.f);
}

TEST_CASE("inverseAffine : ramene un point transforme a sa place")
{
    const Mat4 m = Mat4::fromTRS({1.f, 2.f, 3.f}, kQ, {2.f, 3.f, 4.f});
    const Vec3 p{-3.f, 2.f, 0.5f};
    const Vec3 back = m.inverseAffine().transformPoint(m.transformPoint(p));
    CHECK(back.x == doctest::Approx(p.x).epsilon(1e-4));
    CHECK(back.y == doctest::Approx(p.y).epsilon(1e-4));
    CHECK(back.z == doctest::Approx(p.z).epsilon(1e-4));
}

TEST_CASE("inverseAffine : rotation pure == transposee")
{
    const Mat4 r = Mat4::rotation(kQ);
    CHECK(r.inverseAffine().nearlyEqual(r.transposed()));
}

TEST_CASE("inverseAffine : echelle nulle -> identite")
{
    CHECK(Mat4::scaling({1.f, 0.f, 1.f}).inverseAffine() == Mat4::identity());
}

TEST_CASE("inverseAffine est FAUSSE sur une projection : il faut inverse()")
{
    const Mat4 p = Mat4::perspective(1.0f, 1.6f, 0.1f, 100.f);
    CHECK_FALSE((p * p.inverseAffine()).nearlyEqual(Mat4::identity(), 1e-3f));
    CHECK((p * *p.inverse()).nearlyEqual(Mat4::identity(), 1e-3f));
}
