#pragma once
#include "easing.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace ee::math {

    template <typename T>
    concept Arithmetic = (std::integral<T> || std::floating_point<T>) && !std::same_as<T, bool>;

    template <typename From, typename To>
    concept LossyConversion = std::floating_point<From> && std::integral<To>;

    namespace detail {
        template <std::size_t N, Arithmetic T>
        struct VectorStorage {
            static_assert(N >= 2 && N <= 4, "Vector: N doit etre 2, 3 ou 4");
        };

        template <Arithmetic T>
        struct VectorStorage<2, T> {
            T x{}, y{};
            static constexpr T VectorStorage::* members[] = {&VectorStorage::x, &VectorStorage::y};
            constexpr bool operator==(const VectorStorage&) const noexcept = default;
        };

        template <Arithmetic T>
        struct VectorStorage<3, T> {
            T x{}, y{}, z{};
            static constexpr T VectorStorage::* members[] = {&VectorStorage::x, &VectorStorage::y, &VectorStorage::z};
            constexpr bool operator==(const VectorStorage&) const noexcept = default;
        };

        template <Arithmetic T>
        struct VectorStorage<4, T> {
            T x{}, y{}, z{}, w{};
            static constexpr T VectorStorage::* members[] = {&VectorStorage::x, &VectorStorage::y, &VectorStorage::z, &VectorStorage::w};
            constexpr bool operator==(const VectorStorage&) const noexcept = default;
        };
    } // namespace detail

    template <std::size_t N, Arithmetic T>
    struct Vector : public detail::VectorStorage<N, T> {

      private:
        template <class F>
        static constexpr Vector apply(F&& _f) noexcept {
            return [&]<std::size_t... I>(std::index_sequence<I...>) { return Vector{_f(I)...}; }(std::make_index_sequence<N>{});
        }

        template <class R = T, class F>
        static constexpr R sum(F&& _f) noexcept {
            return [&]<std::size_t... I>(std::index_sequence<I...>) { return (R(_f(I)) + ...); }(std::make_index_sequence<N>{});
        }

      public:
        using Base = detail::VectorStorage<N, T>;
        using WideType = std::conditional_t<std::floating_point<T>, T, std::conditional_t<std::signed_integral<T>, std::int64_t, std::uint64_t>>;

        constexpr Vector() noexcept = default;

        template <std::convertible_to<T>... Args>
        requires(sizeof...(Args) == N)
        explicit((LossyConversion<Args, T> || ...)) constexpr Vector(Args... _a) noexcept : Base{static_cast<T>(_a)...} {}

        template <Arithmetic U>
        explicit(LossyConversion<U, T>) constexpr Vector(const Vector<N, U>& _o) noexcept : Vector(apply([&](std::size_t _i) { return static_cast<T>(_o[_i]); })) {}

        [[nodiscard]] constexpr T& operator[](std::size_t _i) noexcept { return this->*Base::members[_i]; }
        [[nodiscard]] constexpr const T& operator[](std::size_t _i) const noexcept { return this->*Base::members[_i]; }

        [[nodiscard]] constexpr std::array<T, N> toArray() const noexcept { return std::bit_cast<std::array<T, N>>(*this); }

        [[nodiscard]] constexpr bool operator==(const Vector&) const noexcept = default;

        // operateur de base
        [[nodiscard]] constexpr Vector operator+(const Vector& _o) const noexcept {
            return apply([&](std::size_t _i) { return (*this)[_i] + _o[_i]; });
        }
        [[nodiscard]] constexpr Vector operator-(const Vector& _o) const noexcept {
            return apply([&](std::size_t _i) { return (*this)[_i] - _o[_i]; });
        }
        [[nodiscard]] constexpr Vector operator*(const Vector& _o) const noexcept {
            return apply([&](std::size_t _i) { return (*this)[_i] * _o[_i]; });
        }
        [[nodiscard]] constexpr Vector operator/(const Vector& _o) const noexcept {
            return apply([&](std::size_t _i) { return (*this)[_i] / _o[_i]; });
        }
        [[nodiscard]] constexpr Vector operator*(T _s) const noexcept {
            return apply([&](std::size_t _i) { return (*this)[_i] * _s; });
        }
        [[nodiscard]] constexpr Vector operator/(T _s) const noexcept;
        [[nodiscard]] constexpr Vector operator-() const noexcept {
            return apply([&](std::size_t _i) { return -(*this)[_i]; });
        }

        [[nodiscard]] friend constexpr Vector operator*(T _s, const Vector& _v) noexcept { return _v * _s; }

        constexpr Vector& operator+=(const Vector& _o) noexcept { return *this = *this + _o; }
        constexpr Vector& operator-=(const Vector& _o) noexcept { return *this = *this - _o; }
        constexpr Vector& operator*=(const Vector& _o) noexcept { return *this = *this * _o; }
        constexpr Vector& operator/=(const Vector& _o) noexcept { return *this = *this / _o; }
        constexpr Vector& operator*=(T _s) noexcept { return *this = *this * _s; }
        constexpr Vector& operator/=(T _s) noexcept { return *this = *this / _s; }

        // fonction mathématiques
        [[nodiscard]] friend constexpr WideType dot(const Vector& _a, const Vector& _b) noexcept {
            return sum<WideType>([&](std::size_t _i) { return WideType(_a[_i]) * WideType(_b[_i]); });
        }

        [[nodiscard]] friend constexpr Vector cross(const Vector& _a, const Vector& _b) noexcept
        requires(N == 3)
        {
            return {_a.y * _b.z - _a.z * _b.y, _a.z * _b.x - _a.x * _b.z, _a.x * _b.y - _a.y * _b.x};
        }

        [[nodiscard]] friend constexpr WideType cross(const Vector& _a, const Vector& _b) noexcept
        requires(N == 2)
        {
            return WideType(_a.x) * WideType(_b.y) - WideType(_a.y) * WideType(_b.x);
        }

        [[nodiscard]] constexpr WideType lengthSq() const noexcept { return dot(*this, *this); }

        [[nodiscard]] constexpr T length() const noexcept
        requires std::floating_point<T>
        {
            return std::sqrt(lengthSq());
        }

        [[nodiscard]] constexpr WideType manhattan() const noexcept {
            return sum<WideType>([&](std::size_t _i) {
                const WideType v = (*this)[_i];
                return v < 0 ? -v : v;
            });
        }

        [[nodiscard]] constexpr T chebyshev() const noexcept;

        [[nodiscard]] constexpr Vector normalized() const noexcept
        requires std::floating_point<T>
        {
            return *this / length();
        }

        template <std::invocable F>
        [[nodiscard]] constexpr Vector normalizedOr(F _callback) const noexcept
        requires std::floating_point<T>;

        constexpr Vector& normalize() noexcept
        requires std::floating_point<T>
        {
            return *this = normalized();
        }

        [[nodiscard]] constexpr Vector<3, T> xyz() const noexcept
        requires(N == 4)
        {
            return {this->x, this->y, this->z};
        }

        [[nodiscard]] friend constexpr T distance(const Vector& _a, const Vector& _b) noexcept
        requires std::floating_point<T>
        {
            return (_b - _a).length();
        }

        [[nodiscard]] friend constexpr WideType distanceSq(const Vector& _a, const Vector& _b) noexcept { return (_b - _a).lengthSq(); }

        [[nodiscard]] friend constexpr Vector lerp(const Vector& _start, const Vector& _end, T _t) noexcept
        requires std::floating_point<T>
        {
            return apply([&](std::size_t _i) { return _start[_i] + (_end[_i] - _start[_i]) * _t; });
        }

        template <EasingFunction<T> F>
        [[nodiscard]] friend constexpr Vector ease(const Vector& _start, const Vector& _end, T _t, F _f) noexcept
        requires std::floating_point<T>
        {
            return lerp(_start, _end, static_cast<T>(_f(_t)));
        }

        [[nodiscard]] friend constexpr bool nearlyEqual(const Vector& _a, const Vector& _b, T _eps = T(1e-5)) noexcept
        requires std::floating_point<T>
        {
            const auto close = [&](std::size_t _i) {
                const T d = _a[_i] - _b[_i];
                return d <= _eps && -d <= _eps;
            };
            return [&]<std::size_t... I>(std::index_sequence<I...>) { return (close(I) && ...); }(std::make_index_sequence<N>{});
        }
    };

    // securité
    static_assert(sizeof(Vector<2, float>) == 2 * sizeof(float));
    static_assert(sizeof(Vector<3, float>) == 3 * sizeof(float));
    static_assert(sizeof(Vector<4, float>) == 4 * sizeof(float));
    static_assert(std::is_standard_layout_v<Vector<3, float>>);
    static_assert(std::is_trivially_copyable_v<Vector<3, float>>);
    // pour le toArray avec le bit_cast
} // namespace ee::math

#include "Vector.inl"
