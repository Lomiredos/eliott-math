#include <doctest/doctest.h>

#include <initializer_list>
#include <type_traits>
#include <utility>

#include "math/Quaternion.hpp"

using Quaternion = ee::math::Quaternion<float>;
using Vec3 = ee::math::Vector<3, float>;

namespace {
    void checkSameRotation(const Quaternion& _a, const Quaternion& _b) {
        const Vec3 axes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (const Vec3& v : axes) {
            const Vec3 ra = _a.rotate(v);
            const Vec3 rb = _b.rotate(v);
            CHECK(ra.x == doctest::Approx(rb.x).epsilon(1e-4));
            CHECK(ra.y == doctest::Approx(rb.y).epsilon(1e-4));
            CHECK(ra.z == doctest::Approx(rb.z).epsilon(1e-4));
        }
    }
} // namespace

TEST_CASE("toEulerDeg : aller-retour avec fromEulerDeg") {
    const Vec3 angles[] = {
        {0, 0, 0}, {30, 0, 0}, {0, 45, 0}, {0, 0, -60}, {10, 20, 30}, {-35, 70, 120}, {170, -80, 5},
    };
    for (const Vec3& e : angles) {
        CAPTURE(e.x);
        CAPTURE(e.y);
        CAPTURE(e.z);
        const Quaternion q = Quaternion::fromEulerDeg(e);
        checkSameRotation(Quaternion::fromEulerDeg(q.toEulerDeg()), q);
    }
}

TEST_CASE("toEulerDeg : angles simples rendus tels quels") {
    const Vec3 r = Quaternion::fromEulerDeg(10.f, 20.f, 30.f).toEulerDeg();
    CHECK(r.x == doctest::Approx(10.f).epsilon(1e-4));
    CHECK(r.y == doctest::Approx(20.f).epsilon(1e-4));
    CHECK(r.z == doctest::Approx(30.f).epsilon(1e-4));
}

TEST_CASE("toEulerDeg : blocage de cardan, Y = +90 et Y = -90") {
    for (float y : {90.f, -90.f}) {
        CAPTURE(y);
        const Quaternion q = Quaternion::fromEulerDeg(25.f, y, 40.f);
        const Vec3 r = q.toEulerDeg();
        CHECK(r.y == doctest::Approx(y).epsilon(1e-3));
        CHECK(r.z == doctest::Approx(0.f));
        checkSameRotation(Quaternion::fromEulerDeg(r), q);
    }
}

TEST_CASE("fromEulerDeg(Vector3) == fromEulerDeg(x, y, z)") {
    const Quaternion a = Quaternion::fromEulerDeg(12.f, -34.f, 56.f);
    const Quaternion b = Quaternion::fromEulerDeg(Vec3{12.f, -34.f, 56.f});
    CHECK(a.w == doctest::Approx(b.w));
    CHECK(a.x == doctest::Approx(b.x));
    CHECK(a.y == doctest::Approx(b.y));
    CHECK(a.z == doctest::Approx(b.z));
}

namespace {
    constexpr float kHalfPi = ee::math::constants::halfPi<float>;
    constexpr float kPi = ee::math::constants::pi<float>;

    bool near(const Vec3& _a, const Vec3& _b) {
        return nearlyEqual(_a, _b, 1e-4f);
    }

    constexpr Quaternion kId = Quaternion::identity();
} // namespace

static_assert(sizeof(Quaternion) == 4 * sizeof(float));
static_assert(std::is_trivially_copyable_v<Quaternion>);
static_assert(kId.w == 1 && kId.x == 0 && kId.y == 0 && kId.z == 0);
static_assert(kId.lengthSq() == 1 && kId.isNormalized());
static_assert(!Quaternion{0, 0, 0, 0.1f}.isNormalized());
static_assert(kId * kId == kId);
static_assert(kId.rotate(Vec3{1, 2, 3}) == Vec3{1, 2, 3});
static_assert(Quaternion{1, 2, 3, 4}.conjugate() == Quaternion{-1, -2, -3, 4});
static_assert(std::is_same_v<decltype(dot(ee::math::Quaternion<double>{}, ee::math::Quaternion<double>{})), double>);
static_assert(!std::is_convertible_v<ee::math::Vector<4, float>, Quaternion>);

TEST_CASE("rotate : rotation de 90 deg autour de Y et directions") {
    const Quaternion q = Quaternion::fromAxisAngle({0, 1, 0}, kHalfPi);
    CHECK(near(q.rotate({0, 0, -1}), {-1, 0, 0}));
    CHECK(near(q.forward(), {-1, 0, 0}));
    CHECK(near(q.back(), {1, 0, 0}));
    CHECK(near(q.right(), {0, 0, -1}));
    CHECK(near(q.left(), {0, 0, 1}));
    CHECK(near(q.up(), {0, 1, 0}));
    CHECK(near(q.down(), {0, -1, 0}));
    CHECK(near(q * Vec3{0, 0, -1}, q.rotate({0, 0, -1})));
}

TEST_CASE("rotate == q * (v, 0) * conjugate(q)") {
    const Quaternion q = Quaternion::fromEulerDeg(30, -70, 120);
    const Vec3 vs[] = {{1, 2, 3}, {-4, 0.5f, 2}, {0, 0, -1}};
    for (const Vec3& v : vs) {
        const Quaternion ref = q * Quaternion{v.x, v.y, v.z, 0} * q.conjugate();
        CHECK(near(q.rotate(v), {ref.x, ref.y, ref.z}));
    }
}

TEST_CASE("produit de Hamilton : a * b applique b puis a") {
    const Quaternion a = Quaternion::fromAxisAngle({0, 1, 0}, kHalfPi);
    const Quaternion b = Quaternion::fromAxisAngle({1, 0, 0}, kHalfPi);
    const Vec3 v{0, 0, -1};
    CHECK(near((a * b).rotate(v), a.rotate(b.rotate(v))));

    Quaternion c = a;
    c *= b;
    CHECK(c == a * b);
}

TEST_CASE("inverse, conjugate, normalize") {
    const Quaternion q = Quaternion::fromEulerDeg(30, -70, 120);
    CHECK(sameRotation(q.inverse() * q, kId));
    CHECK(sameRotation(q * q.conjugate(), kId));

    const Quaternion big = q * 3.f;
    CHECK(sameRotation(big.inverse() * big, kId));
    CHECK_FALSE(big.isNormalized());
    CHECK(big.normalized().isNormalized());
    CHECK(sameRotation(big.normalized(), q));

    Quaternion n{0, 0, 0, 2};
    n.normalize();
    CHECK(n == kId);
}

TEST_CASE("sameRotation et angleBetween") {
    const Quaternion q = Quaternion::fromEulerDeg(30, -70, 120);
    CHECK(sameRotation(q, q));
    CHECK(sameRotation(-q, q));
    CHECK_FALSE(sameRotation(q, kId));

    const Quaternion r = Quaternion::fromAxisAngle({0, 1, 0}, kHalfPi);
    CHECK(angleBetween(kId, r) == doctest::Approx(kHalfPi));
    CHECK(angleBetween(kId, -r) == doctest::Approx(kHalfPi));
    CHECK(angleBetween(q, q) == doctest::Approx(0.f).epsilon(1e-3));
}

TEST_CASE("fromBasis : aller-retour avec right, up, back") {
    const Quaternion qs[] = {kId,
                             Quaternion::fromEulerDeg(30, -70, 120),
                             Quaternion::fromEulerDeg(170, 10, -175),
                             Quaternion::fromAxisAngle({1, 0, 0}, kPi),
                             Quaternion::fromAxisAngle({0, 1, 0}, kPi),
                             Quaternion::fromAxisAngle({0, 0, 1}, kPi)};
    for (const Quaternion& q : qs)
        CHECK(sameRotation(Quaternion::fromBasis(q.right(), q.up(), q.back()), q));
}

TEST_CASE("lookRotation : forward suit la direction, up reste en haut") {
    const Vec3 dirs[] = {{1, 0, 0}, {0, 0, 1}, {0, 0, -1}, {1, 1, 1}, {-3, 0.5f, 2}, {0, 1, 0}, {0, -1, 0}};
    for (const Vec3& d : dirs) {
        const Quaternion q = Quaternion::lookRotation(d);
        CHECK(near(q.forward(), d.normalized()));
        CHECK(q.isNormalized(1e-4f));
    }
    CHECK(near(Quaternion::lookRotation({1, 0, 0}).up(), {0, 1, 0}));
    CHECK(near(Quaternion::lookRotation({0, 0, -1}).up(), {0, 1, 0}));
}

TEST_CASE("fromTo : amene from sur to, y compris a 180 deg") {
    const std::pair<Vec3, Vec3> cases[] = {
        {{1, 0, 0}, {0, 1, 0}}, {{1, 2, 3}, {-3, 0, 1}}, {{1, 0, 0}, {-1, 0, 0}}, {{0, 1, 0}, {0, -1, 0}}, {{0, 0, 2}, {0, 0, 5}},
    };
    for (const auto& [f, t] : cases) {
        const Quaternion q = Quaternion::fromTo(f, t);
        CHECK(near(q.rotate(f.normalized()), t.normalized()));
        CHECK(q.isNormalized(1e-4f));
    }
}

TEST_CASE("toAxisAngle") {
    const Quaternion q = Quaternion::fromAxisAngle({0, 1, 0}, kHalfPi);
    const auto aa = q.toAxisAngle();
    CHECK(near(aa.axis, {0, 1, 0}));
    CHECK(aa.angle == doctest::Approx(kHalfPi));

    const auto [axis, angle] = (-q).toAxisAngle();
    CHECK(near(axis, {0, 1, 0}));
    CHECK(angle == doctest::Approx(kHalfPi));

    CHECK(kId.toAxisAngle().angle == 0.f);

    const Quaternion e = Quaternion::fromEulerDeg(30, -70, 120);
    const auto ea = e.toAxisAngle();
    CHECK(sameRotation(Quaternion::fromAxisAngle(ea.axis, ea.angle), e));
}

TEST_CASE("slerp : extremites, vitesse constante, chemin le plus court") {
    const Quaternion b = Quaternion::fromAxisAngle({0, 1, 0}, kHalfPi);
    CHECK(sameRotation(slerp(kId, b, 0.f), kId));
    CHECK(sameRotation(slerp(kId, b, 1.f), b));
    for (float t : {0.1f, 0.25f, 0.5f, 0.9f})
        CHECK(angleBetween(kId, slerp(kId, b, t)) == doctest::Approx(kHalfPi * t));
    CHECK(sameRotation(slerp(kId, -b, 0.5f), slerp(kId, b, 0.5f)));

    const Quaternion close = Quaternion::fromAxisAngle({0, 1, 0}, 1e-3f);
    CHECK(slerp(kId, close, 0.5f).isNormalized(1e-4f));
}

TEST_CASE("nlerp : normalise et prend le chemin le plus court") {
    const Quaternion b = Quaternion::fromAxisAngle({0, 1, 0}, kHalfPi);
    CHECK(nlerp(kId, b, 0.5f).isNormalized(1e-5f));
    CHECK(angleBetween(kId, nlerp(kId, b, 0.5f)) == doctest::Approx(kHalfPi * 0.5f));
    CHECK(angleBetween(kId, nlerp(kId, -b, 0.5f)) == doctest::Approx(kHalfPi * 0.5f));
}

TEST_CASE("rotateTowards : pas de depassement") {
    const Quaternion b = Quaternion::fromAxisAngle({0, 1, 0}, kHalfPi);
    const float step = kPi / 18.f;
    CHECK(angleBetween(kId, rotateTowards(kId, b, step)) == doctest::Approx(step));
    CHECK(sameRotation(rotateTowards(kId, b, kPi), b));
    CHECK(sameRotation(rotateTowards(b, b, step), b));
}

TEST_CASE("Quaternion<double>") {
    using Qd = ee::math::Quaternion<double>;
    const Qd q = Qd::fromEulerDeg(30.0, -70.0, 120.0);
    CHECK(q.isNormalized(1e-12));
    CHECK(sameRotation(Qd::fromEulerDeg(q.toEulerDeg()), q, 1e-12));
}
