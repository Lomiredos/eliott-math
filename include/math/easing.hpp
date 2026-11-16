#pragma once
#include <concepts>
#include <type_traits>

namespace ee::math {

    template <typename F, typename T>
    concept EasingFunction = std::invocable<F, T> && std::convertible_to<std::invoke_result_t<F, T>, T>;

    namespace easing {

        template <std::floating_point T>
        constexpr T smoothstep(T _t) noexcept {
            return _t * _t * (T(3) - T(2) * _t);
        }
        template <std::floating_point T>
        constexpr T smootherstep(T _t) noexcept {
            return _t * _t * _t * (_t * (_t * T(6) - T(15)) + T(10));
        }
        template <std::floating_point T>
        constexpr T inQuad(T _t) noexcept {
            return _t * _t;
        }
        template <std::floating_point T>
        constexpr T outQuad(T _t) noexcept {
            return _t * (T(2) - _t);
        }
    } // namespace easing
} // namespace ee::math