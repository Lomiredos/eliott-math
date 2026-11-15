#pragma once
#include <array>
#include <bit>
#include <type_traits>
#include <concepts>
#include <cstddef>

namespace ee::math {

    template<typename T>
    concept Arithmetic = (std::integral<T> || std::floating_point<T>) && !std::same_as<T, bool>;
    

    namespace detail {
        template<std::size_t N, Arithmetic T> struct VectorStorage;

        template<Arithmetic T> struct VectorStorage<2, T>
        {
            T x{}, y{};
            static constexpr T VectorStorage::* members[] = {&VectorStorage::x, &VectorStorage::y};
            constexpr bool operator==(const VectorStorage&) const noexcept = default;  
        };
        
        template<Arithmetic T> struct VectorStorage<3, T>
        {
            T x{}, y{}, z{};
            static constexpr T VectorStorage::* members[] = {&VectorStorage::x, &VectorStorage::y, &VectorStorage::z};
            constexpr bool operator==(const VectorStorage&) const noexcept = default;  
        };

        template<Arithmetic T> struct VectorStorage<4, T>
        {
            T x{}, y{}, z{}, w{};
            static constexpr T VectorStorage::* members[] = {&VectorStorage::x, &VectorStorage::y, &VectorStorage::z, &VectorStorage::w};
            constexpr bool operator==(const VectorStorage&) const noexcept = default;  
        };
    }

    
    template<std::size_t N, Arithmetic T>
    requires (N >= 2 && N <= 4)
    struct Vector : public detail::VectorStorage<N, T> {
        using Base = detail::VectorStorage<N, T>;


        constexpr Vector() noexcept = default;


        template<std::convertible_to<T>... Args>
            requires(sizeof...(Args) == N)
        constexpr Vector(Args... _a) noexcept : Base{ static_cast<T>(_a)...} {}

        [[nodiscard]] constexpr T& operator[](std::size_t _i) noexcept {return this->*Base::members[_i];}
        [[nodiscard]] constexpr const T& operator[](std::size_t _i) const noexcept {return this->*Base::members[_i];}


        [[nodiscard]] constexpr std::array<T, N> toArray() const noexcept {
            return std::bit_cast<std::array<T, N>>(*this);
        }
    };








    static_assert(sizeof(Vector<2, float>) == 2 * sizeof(float));
    static_assert(sizeof(Vector<3, float>) == 3 * sizeof(float)); // s'assurer qu'on a pas de prise de memoir cacher
    static_assert(sizeof(Vector<4, float>) == 4 * sizeof(float));
    static_assert(std::is_standard_layout_v<Vector<3, float>>);
    static_assert(std::is_trivially_copyable_v<Vector<3, float>>); // pour le toArray avec le bit_cast
}