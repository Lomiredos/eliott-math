#include <doctest/doctest.h>

#include <limits>
#include <utility>

#include "math/Matrix.hpp"

using ee::math::Matrix;
using Mat2 = Matrix<2, float>;
using Mat3 = Matrix<3, float>;
using Mat4 = Matrix<4, float>;

namespace
{
    constexpr Mat4 sequence()
    {
        Mat4 r{};
        for (std::size_t i = 0; i < 16; ++i)
            r.m[i] = static_cast<float>(i);
        return r;
    }
}

static_assert(Mat4::identity()(0, 0) == 1.f);
static_assert(Mat4::identity()(0, 1) == 0.f);
static_assert(Mat4::identity() == Mat4::identity());
static_assert(Mat4{} != Mat4::identity());
static_assert(sequence()(1, 0) == 1.f);
static_assert(sequence()(0, 1) == 4.f);
static_assert(sequence() * Mat4::identity() == sequence());
static_assert(Mat4::identity() * sequence() == sequence());
static_assert(sequence().transposed().transposed() == sequence());
static_assert(sequence().transposed()(1, 3) == sequence()(3, 1));

TEST_CASE("identite : 1 sur la diagonale, 0 ailleurs")
{
    const Mat4 i4 = Mat4::identity();
    for (std::size_t r = 0; r < 4; ++r)
        for (std::size_t c = 0; c < 4; ++c)
            CHECK(i4(r, c) == (r == c ? 1.f : 0.f));

    const Mat3 i3 = Mat3::identity();
    CHECK(i3(2, 2) == 1.f);
    CHECK(i3(0, 2) == 0.f);

    const Mat2 i2 = Mat2::identity();
    CHECK(i2(1, 1) == 1.f);
    CHECK(i2(1, 0) == 0.f);
}

TEST_CASE("matrice par defaut : que des zeros")
{
    const Mat4 z{};
    for (float v : z.m)
        CHECK(v == 0.f);
}

TEST_CASE("stockage par colonnes : (row, col) -> m[col * N + row]")
{
    Mat4 a{};
    a(0, 1) = 5.f;
    CHECK(a.m[4] == 5.f);
    CHECK(a.m[1] == 0.f);

    a(3, 0) = 7.f;
    CHECK(a.m[3] == 7.f);

    a(2, 3) = 9.f;
    CHECK(a.m[14] == 9.f);

    Mat3 b{};
    b(1, 2) = 4.f;
    CHECK(b.m[7] == 4.f);
}

TEST_CASE("translation dans la 4e colonne = m[12], m[13], m[14] (ordre OpenGL)")
{
    Mat4 t = Mat4::identity();
    t(0, 3) = 1.f;
    t(1, 3) = 2.f;
    t(2, 3) = 3.f;
    CHECK(t.m[12] == 1.f);
    CHECK(t.m[13] == 2.f);
    CHECK(t.m[14] == 3.f);
    CHECK(t.m[15] == 1.f);
}

TEST_CASE("data() pointe sur m")
{
    Mat4 a = sequence();
    CHECK(a.data() == a.m);
    CHECK(a.data()[5] == 5.f);

    const Mat4 &ca = a;
    CHECK(ca.data()[15] == 15.f);
}

TEST_CASE("== et != : egalite exacte")
{
    Mat4 a = sequence();
    Mat4 b = sequence();
    CHECK(a == b);

    b(2, 2) += 1e-6f;
    CHECK(a != b);
}

TEST_CASE("nearlyEqual : tolerance")
{
    Mat4 a = sequence();
    Mat4 b = a;
    b(1, 1) += 1e-6f;
    CHECK(a.nearlyEqual(b));
    CHECK_FALSE(a == b);

    b(1, 1) += 1e-3f;
    CHECK_FALSE(a.nearlyEqual(b));
    CHECK(a.nearlyEqual(b, 1e-2f));
}

TEST_CASE("nearlyEqual : un NaN n'est jamais egal")
{
    Mat4 a = Mat4::identity();
    Mat4 b = a;
    b(0, 0) = std::numeric_limits<float>::quiet_NaN();
    CHECK_FALSE(a.nearlyEqual(b));
    CHECK_FALSE(b.nearlyEqual(b));
}

TEST_CASE("Matrix<4, double>")
{
    using Mat4d = Matrix<4, double>;
    const Mat4d i = Mat4d::identity();
    CHECK(i(3, 3) == 1.0);
    CHECK(i.nearlyEqual(Mat4d::identity()));
}

TEST_CASE("produit 2x2 calcule a la main")
{
    Mat2 a{}; a(0,0) = 1; a(0,1) = 2; a(1,0) = 3; a(1,1) = 4;
    Mat2 b{}; b(0,0) = 5; b(0,1) = 6; b(1,0) = 7; b(1,1) = 8;
    const Mat2 p = a * b;
    CHECK(p(0,0) == 19.f); CHECK(p(0,1) == 22.f);
    CHECK(p(1,0) == 43.f); CHECK(p(1,1) == 50.f);
}

TEST_CASE("transposer 2x2 calcule a la main")
{
    Mat2 a{}; a(0,0) = 1; a(0,1) = 2; a(1,0) = 3; a(1,1) = 4;
    const Mat2 at = a.transposed(); 
    CHECK(at(0, 0) == 1.f); CHECK(at(0, 1) == 3.f);
    CHECK(at(1, 1) == 4.f); CHECK(at(1, 0) == 2.f);
}

TEST_CASE("(A * B)^T == B^T * A^T")
{
    const Mat4 a = sequence();
    Mat4 b = Mat4::identity();
    b(0, 3) = 2.f; b(2, 1) = -1.f; b(3, 0) = 0.5f;
    CHECK((a * b).transposed() == b.transposed() * a.transposed());
}
namespace
{
    constexpr Mat4 filled(float _v)
    {
        Mat4 r{};
        for (float &x : r.m)
            x = _v;
        return r;
    }
}

static_assert(sequence() + filled(1.f) == [] { Mat4 r = sequence(); for (float &x : r.m) x += 1.f; return r; }());
static_assert(sequence() - sequence() == Mat4{});
static_assert(sequence() * 2.f == sequence() + sequence());
static_assert(2.f * sequence() == sequence() * 2.f);
static_assert([] { Mat4 a = sequence(); a *= Mat4::identity(); return a; }() == sequence());

static_assert(noexcept(Mat4{} == Mat4{}));
static_assert(noexcept(Mat4{}.nearlyEqual(Mat4{})));
static_assert(noexcept(Mat4{} + Mat4{}));
static_assert(noexcept(Mat4{} - Mat4{}));
static_assert(noexcept(Mat4{} * Mat4{}));
static_assert(noexcept(Mat4{} * 2.f));
static_assert(noexcept(2.f * Mat4{}));
static_assert(noexcept(Mat4{}.transposed()));
static_assert(noexcept(std::declval<Mat4 &>() += Mat4{}));
static_assert(noexcept(std::declval<Mat4 &>() -= Mat4{}));
static_assert(noexcept(std::declval<Mat4 &>() *= Mat4{}));
static_assert(noexcept(std::declval<Mat4 &>() *= 2.f));

TEST_CASE("+ et - case par case")
{
    const Mat4 a = sequence();
    const Mat4 b = filled(10.f);

    const Mat4 s = a + b;
    const Mat4 d = a - b;
    for (std::size_t i = 0; i < 16; ++i)
    {
        CHECK(s.m[i] == a.m[i] + 10.f);
        CHECK(d.m[i] == a.m[i] - 10.f);
    }
    CHECK(a == sequence());
}

TEST_CASE("+= et -= modifient et renvoient la matrice")
{
    Mat4 a = sequence();
    Mat4 &ref = (a += filled(1.f));
    CHECK(&ref == &a);
    CHECK(a(0, 0) == 1.f);
    CHECK(a(3, 3) == 16.f);

    (a -= filled(1.f)) -= filled(1.f);
    CHECK(a(0, 0) == -1.f);
    CHECK(a(3, 3) == 14.f);
}

TEST_CASE("multiplication par un scalaire, dans les deux sens")
{
    const Mat4 a = sequence();
    const Mat4 l = a * 3.f;
    const Mat4 r = 3.f * a;
    for (std::size_t i = 0; i < 16; ++i)
    {
        CHECK(l.m[i] == a.m[i] * 3.f);
        CHECK(r.m[i] == a.m[i] * 3.f);
    }

    const Mat4 fromInt = 2 * a;
    CHECK(fromInt == a * 2.f);
}

TEST_CASE("*= scalaire")
{
    Mat4 a = sequence();
    Mat4 &ref = (a *= 0.5f);
    CHECK(&ref == &a);
    for (std::size_t i = 0; i < 16; ++i)
        CHECK(a.m[i] == static_cast<float>(i) * 0.5f);
}

TEST_CASE("*= matrice == a * b, sans lire de cases deja modifiees")
{
    Mat2 a{}; a(0,0) = 1; a(0,1) = 2; a(1,0) = 3; a(1,1) = 4;
    Mat2 b{}; b(0,0) = 5; b(0,1) = 6; b(1,0) = 7; b(1,1) = 8;

    Mat2 c = a;
    Mat2 &ref = (c *= b);
    CHECK(&ref == &c);
    CHECK(c == a * b);
    CHECK(c(0,0) == 19.f); CHECK(c(0,1) == 22.f);
    CHECK(c(1,0) == 43.f); CHECK(c(1,1) == 50.f);

    Mat4 s = sequence();
    s *= s;
    CHECK(s == sequence() * sequence());
}

TEST_CASE("produit 4x4 : A * (B * C) == (A * B) * C")
{
    const Mat4 a = sequence();
    Mat4 b = Mat4::identity();
    b(0, 3) = 2.f; b(2, 1) = -1.f;
    const Mat4 c = sequence().transposed() * 0.1f;
    CHECK((a * (b * c)).nearlyEqual((a * b) * c, 1e-3f));
}

TEST_CASE("Matrix<3, float> : operations generiques")
{
    Mat3 a{};
    for (std::size_t i = 0; i < 9; ++i)
        a.m[i] = static_cast<float>(i + 1);

    CHECK(a * Mat3::identity() == a);
    CHECK(a - a == Mat3{});
    CHECK(a.transposed()(0, 2) == a(2, 0));
    CHECK((a + a) == 2.f * a);
}
