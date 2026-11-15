#include <doctest/doctest.h>

#include "math/Vector.hpp"

using namespace ee::math;

static_assert(Vector<3, float>{1, 2, 3}[1] == 2);