#pragma once
#include "Vector.hpp"

namespace ee::math {

    template <std::size_t N, Arithmetic T>
    constexpr Vector<N, T> Vector<N, T>::operator/(T _s) const noexcept {
        if constexpr (std::floating_point<T>) {
            const T inv = T(1) / _s;
            return apply([&](std::size_t _i) { return (*this)[_i] * inv; });
        } else {
            return apply([&](std::size_t _i) { return (*this)[_i] / _s; });
        }
    }

    template <std::size_t N, Arithmetic T>
    constexpr T Vector<N, T>::chebyshev() const noexcept {
        T m{};
        for (std::size_t i = 0; i < N; ++i) {
            const T v = (*this)[i] < T(0) ? -(*this)[i] : (*this)[i];
            m = v > m ? v : m;
        }
        return m;
    }

    template <std::size_t N, Arithmetic T>
    template <std::invocable F>
    constexpr Vector<N, T> Vector<N, T>::normalizedOr(F _callback) const noexcept
    requires std::floating_point<T>
    {
        if (lengthSq() == T(0)) {
            _callback();
            return Vector{};
        }
        return normalized();
    }

} // namespace ee::math
