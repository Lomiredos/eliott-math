#pragma once
#include "Quaternion.hpp"
#include "Vector.hpp"
#include <cmath>
#include <concepts>
#include <cstddef>
#include <optional>

namespace ee::math {

    template <std::size_t N, std::floating_point T>
    requires(N >= 2)
    class Matrix {
      private:
        [[nodiscard]] constexpr T detPivotGauss() const noexcept;

      public:
        T m[N * N]{};

        [[nodiscard]] static constexpr Matrix identity() noexcept;

        [[nodiscard]] constexpr T& operator()(std::size_t _row, std::size_t _col) noexcept;
        [[nodiscard]] constexpr T operator()(std::size_t _row, std::size_t _col) const noexcept;

        [[nodiscard]] constexpr bool operator==(const Matrix&) const = default;
        [[nodiscard]] constexpr bool nearlyEqual(const Matrix& _o, T _eps = T(1e-5)) const noexcept;

        [[nodiscard]] constexpr const T* data() const noexcept { return m; }
        [[nodiscard]] constexpr T* data() noexcept { return m; }

        [[nodiscard]] constexpr Matrix operator*(const Matrix& _o) const noexcept;

        [[nodiscard]] constexpr Matrix transposed() const noexcept;

        constexpr Matrix& operator+=(const Matrix& _o) noexcept;
        constexpr Matrix& operator-=(const Matrix& _o) noexcept;
        constexpr Matrix& operator*=(T _t) noexcept;
        constexpr Matrix& operator*=(const Matrix& _o) noexcept;

        [[nodiscard]] constexpr Matrix operator+(const Matrix& _o) const noexcept;
        [[nodiscard]] constexpr Matrix operator-(const Matrix& _o) const noexcept;
        [[nodiscard]] constexpr Matrix operator*(T _t) const noexcept;
        [[nodiscard]] friend constexpr Matrix operator*(T _t, const Matrix& _m) noexcept { return _m * _t; }

        [[nodiscard]] constexpr Vector<N, T> operator*(const Vector<N, T>& _v) const noexcept
        requires(N <= 4);

        [[nodiscard]] Quaternion<T> toQuaternion() const noexcept
        requires(N == 3);

        // fonction reserver pour les 4x4 (ou 3x3 pour rotation)
        // transformation monde
        [[nodiscard]] static constexpr Matrix translation(const Vector<3, T>& _t) noexcept
        requires(N == 4);
        [[nodiscard]] static constexpr Matrix scaling(const Vector<3, T>& _s) noexcept
        requires(N == 4);
        [[nodiscard]] static Matrix rotation(const Quaternion<T>& _q) noexcept
        requires(N == 4 || N == 3);
        [[nodiscard]] static Matrix fromTRS(const Vector<3, T>& _t, const Quaternion<T>& _r, const Vector<3, T>& _s) noexcept
        requires(N == 4);

        // fonction reserver pour les 4x4
        // camera
        [[nodiscard]] static constexpr Matrix perspective(T _fovYRad, T _aspect, T _near, T _far) noexcept
        requires(N == 4);
        [[nodiscard]] static constexpr Matrix ortho(T _l, T _r, T _b, T _t, T _near, T _far) noexcept
        requires(N == 4);
        [[nodiscard]] static constexpr Matrix lookAt(const Vector<3, T>& _eye, const Vector<3, T>& _center, const Vector<3, T>& _up) noexcept
        requires(N == 4);

        // fonction reserver pour les 4x4
        // transformation
        [[nodiscard]] constexpr Vector<3, T> getTranslation() const noexcept
        requires(N == 4);
        [[nodiscard]] constexpr Vector<3, T> transformPoint(const Vector<3, T>& _p) const noexcept
        requires(N == 4);
        [[nodiscard]] constexpr Vector<3, T> projectPoint(const Vector<3, T>& _p) const noexcept
        requires(N == 4);
        [[nodiscard]] constexpr Vector<3, T> transformDir(const Vector<3, T>& _d) const noexcept
        requires(N == 4);
        [[nodiscard]] constexpr Matrix<3, T> upperLeft3() const noexcept
        requires(N == 4);

        [[nodiscard]] constexpr Matrix<3, T> normalMatrix() const noexcept
        requires(N == 4);

        [[nodiscard]] constexpr T determinant() const noexcept;

        [[nodiscard]] constexpr std::optional<Matrix> inverse(T _eps = T(1e-5)) const noexcept;

        [[nodiscard]] constexpr Matrix inverseAffine() const noexcept
        requires(N == 4);

        [[nodiscard]] bool decompose(Vector<3, T>& _t, Quaternion<T>& _r, Vector<3, T>& _s, T _eps = T(1e-5)) const noexcept
        requires(N == 4);
    };

} // namespace ee::math

#include "Matrix.inl"