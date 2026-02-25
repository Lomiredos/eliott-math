#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "math/math.hpp"

using namespace ee::math;

TEST_CASE("Fonction isInside"){

    Rect<int> a = Rect<int>(16, 16, 30, 30);

    CHECK(a.isInside(24, 24));
    CHECK(a.isInside(15, 24) == false);
}


TEST_CASE("Fonction Intersects"){

    Rect<int> a = Rect<int>(16, 16, 30, 30);
    
    Rect<int> b = Rect<int>(24, 24, 19, 19);

    Rect<int> c = Rect<int>(2, 2, 2, 2);

    CHECK(a.Intersects(b));
    CHECK(a.Intersects(c) == false);
}


TEST_CASE("Fonction Magnetude"){
    Vector2<int> a =Vector2<int>(3, 4);

    CHECK(a.Magnetude() == 5);
    CHECK(a.Magnetude() != 6);
}