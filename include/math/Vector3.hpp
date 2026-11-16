#pragma once
#include "Vector.hpp"

namespace ee::math {
    template <Arithmetic T>
    using Vector3 [[deprecated("use Vector<3, T> instead")]] = Vector<3, T>;
}