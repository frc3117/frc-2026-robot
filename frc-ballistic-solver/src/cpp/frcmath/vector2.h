#ifndef VECTOR2_H
#define VECTOR2_H

#include <pybind11/pybind11.h>
namespace py = pybind11;

void initVector2(py::module &m);

namespace FRC {
    struct Vector2 {
        float x;
        float y;

        Vector2();
        Vector2(float x, float y);

        float sqrMagnitude();
        float magnitude();

        float dot(const Vector2& other);

        friend Vector2 operator+(const Vector2& lhs, const Vector2& rhs);
        friend Vector2 operator+(const Vector2& lhs, const float& rhs);
        friend Vector2 operator+(const Vector2& lhs, const int& rhs);
        Vector2& operator+=(const Vector2& rhs);
        Vector2& operator+=(const float& rhs);
        Vector2& operator+=(const int& rhs);

        friend Vector2 operator-(const Vector2 lhs, const Vector2& rhs);
        friend Vector2 operator-(const Vector2 lhs, const float& rhs);
        friend Vector2 operator-(const Vector2& lhs, const int& rhs);
        Vector2& operator-=(const Vector2& rhs);
        Vector2& operator-=(const float& rhs);
        Vector2& operator-=(const int& rhs);

        friend Vector2 operator*(const Vector2& lhs, const Vector2& rhs);
        friend Vector2 operator*(const Vector2& lhs, const float& rhs);
        friend Vector2 operator*(const Vector2& lhs, const int& rhs);
        Vector2& operator*=(const Vector2& rhs);
        Vector2& operator*=(const float& rhs);
        Vector2& operator*=(const int& rhs);

        friend Vector2 operator/(const Vector2& lhs, const Vector2& rhs);
        friend Vector2 operator/(const Vector2& lhs, const float& rhs);
        friend Vector2 operator/(const Vector2& lhs, const int& rhs);
        Vector2& operator/=(const Vector2& rhs);
        Vector2& operator/=(const float& rhs);
        Vector2& operator/=(const int& rhs);
    };
}
#endif