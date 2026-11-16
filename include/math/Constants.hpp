#pragma once
#include <concepts>
#include <numbers>

namespace ee::math::constants {
    template <std::floating_point T> inline constexpr T pi = std::numbers::pi_v<T>;
    template <std::floating_point T> inline constexpr T halfPi = pi<T> / T(2);
    template <std::floating_point T> inline constexpr T twoPi = pi<T> * T(2);
    template <std::floating_point T> inline constexpr T piSq = pi<T> * pi<T>;
    template <std::floating_point T> inline constexpr T degToRad = pi<T> / T(180);
    template <std::floating_point T> inline constexpr T radToDeg = T(180) / pi<T>;
}
