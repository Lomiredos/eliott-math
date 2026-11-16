#pragma once
#include "Vector.hpp"

namespace ee::math {
    template <Arithmetic T>
    using Vector2 [[deprecated("use Vector<2, T> instead")]] = Vector<2, T>;
}