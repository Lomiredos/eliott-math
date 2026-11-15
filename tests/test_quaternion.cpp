#include <doctest/doctest.h>

#include <initializer_list>

#include "math/Quaternion.hpp"

using ee::math::Quaternion;
using Vec3 = ee::math::Vector3<float>;

namespace
{
    void checkSameRotation(const Quaternion &_a, const Quaternion &_b)
    {
        const Vec3 axes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (const Vec3 &v : axes)
        {
            const Vec3 ra = _a.rotate(v);
            const Vec3 rb = _b.rotate(v);
            CHECK(ra.x == doctest::Approx(rb.x).epsilon(1e-4));
            CHECK(ra.y == doctest::Approx(rb.y).epsilon(1e-4));
            CHECK(ra.z == doctest::Approx(rb.z).epsilon(1e-4));
        }
    }
}

TEST_CASE("toEulerDeg : aller-retour avec fromEulerDeg")
{
    const Vec3 angles[] = {
        {0, 0, 0}, {30, 0, 0}, {0, 45, 0}, {0, 0, -60},
        {10, 20, 30}, {-35, 70, 120}, {170, -80, 5},
    };
    for (const Vec3 &e : angles)
    {
        CAPTURE(e.x);
        CAPTURE(e.y);
        CAPTURE(e.z);
        const Quaternion q = Quaternion::fromEulerDeg(e);
        checkSameRotation(Quaternion::fromEulerDeg(q.toEulerDeg()), q);
    }
}

TEST_CASE("toEulerDeg : angles simples rendus tels quels")
{
    const Vec3 r = Quaternion::fromEulerDeg(10.f, 20.f, 30.f).toEulerDeg();
    CHECK(r.x == doctest::Approx(10.f).epsilon(1e-4));
    CHECK(r.y == doctest::Approx(20.f).epsilon(1e-4));
    CHECK(r.z == doctest::Approx(30.f).epsilon(1e-4));
}

TEST_CASE("toEulerDeg : blocage de cardan, Y = +90 et Y = -90")
{
    for (float y : {90.f, -90.f})
    {
        CAPTURE(y);
        const Quaternion q = Quaternion::fromEulerDeg(25.f, y, 40.f);
        const Vec3 r = q.toEulerDeg();
        CHECK(r.y == doctest::Approx(y).epsilon(1e-3));
        CHECK(r.z == doctest::Approx(0.f));
        checkSameRotation(Quaternion::fromEulerDeg(r), q);
    }
}

TEST_CASE("fromEulerDeg(Vector3) == fromEulerDeg(x, y, z)")
{
    const Quaternion a = Quaternion::fromEulerDeg(12.f, -34.f, 56.f);
    const Quaternion b = Quaternion::fromEulerDeg(Vec3{12.f, -34.f, 56.f});
    CHECK(a.w == doctest::Approx(b.w));
    CHECK(a.x == doctest::Approx(b.x));
    CHECK(a.y == doctest::Approx(b.y));
    CHECK(a.z == doctest::Approx(b.z));
}
