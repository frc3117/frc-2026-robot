#include "vector2.h"

#include <cmath>

#include <pybind11/pybind11.h>
namespace py = pybind11;

FRC::Vector2::Vector2() {
    x = 0.0f;
    y = 0.0f;
}
FRC::Vector2::Vector2(float x, float y) {
    this->x = x;
    this->y = y;
}

float FRC::Vector2::sqrMagnitude() {
    return (x * x) + (y * y);
}
float FRC::Vector2::magnitude() {
    return std::sqrt(sqrMagnitude());
}

float FRC::Vector2::dot(const FRC::Vector2& other) {
    return (x * other.x) + (y * other.y);
}

FRC::Vector2 operator+(const FRC::Vector2& lhs, const FRC::Vector2& rhs) {
    return FRC::Vector2(lhs.x + rhs.x, lhs.y + rhs.y);
}
FRC::Vector2 operator+(const FRC::Vector2& lhs, const float& rhs) {
    return FRC::Vector2(lhs.x + rhs, lhs.y + rhs);
}
FRC::Vector2 operator+(const FRC::Vector2& lhs, const int& rhs) {
    return FRC::Vector2(lhs.x + rhs, lhs.y + rhs);
}
FRC::Vector2& FRC::Vector2::operator+=(const FRC::Vector2& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
}
FRC::Vector2& FRC::Vector2::operator+=(const float& rhs) {
    x += rhs;
    y += rhs;
    return *this;
}
FRC::Vector2& FRC::Vector2::operator+=(const int& rhs) {
    x += rhs;
    y += rhs;
    return *this;
}

FRC::Vector2 operator-(const FRC::Vector2& lhs, const FRC::Vector2& rhs) {
    return FRC::Vector2(lhs.x - rhs.x, lhs.y - rhs.y);
}
FRC::Vector2 operator-(const FRC::Vector2& lhs, const float& rhs) {
    return FRC::Vector2(lhs.x - rhs, lhs.y - rhs);
}
FRC::Vector2 operator-(const FRC::Vector2& lhs, const int& rhs) {
    return FRC::Vector2(lhs.x - rhs, lhs.y - rhs);
}
FRC::Vector2& FRC::Vector2::operator-=(const FRC::Vector2& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}
FRC::Vector2& FRC::Vector2::operator-=(const float& rhs) {
    x -= rhs;
    y -= rhs;
    return *this;
}
FRC::Vector2& FRC::Vector2::operator-=(const int& rhs) {
    x -= rhs;
    y -= rhs;
    return *this;
}

FRC::Vector2 operator*(const FRC::Vector2& lhs, const FRC::Vector2& rhs) {
    return FRC::Vector2(lhs.x * rhs.x, lhs.y * rhs.y);
}
FRC::Vector2 operator*(const FRC::Vector2& lhs, const float& rhs) {
    return FRC::Vector2(lhs.x * rhs, lhs.y * rhs);
}
FRC::Vector2 operator*(const FRC::Vector2& lhs, const int& rhs) {
    return FRC::Vector2(lhs.x * rhs, lhs.y * rhs);
}
FRC::Vector2& FRC::Vector2::operator*=(const FRC::Vector2& rhs) {
    x *= rhs.x;
    y *= rhs.y;
    return *this;
}
FRC::Vector2& FRC::Vector2::operator*=(const float& rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
}
FRC::Vector2& FRC::Vector2::operator*=(const int& rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
}

FRC::Vector2 operator/(const FRC::Vector2& lhs, const FRC::Vector2& rhs) {
    return FRC::Vector2(lhs.x / rhs.x, lhs.y / rhs.y);
}
FRC::Vector2 operator/(const FRC::Vector2& lhs, const float& rhs) {
    return FRC::Vector2(lhs.x / rhs, lhs.y + rhs);
}
FRC::Vector2 operator/(const FRC::Vector2& lhs, const int& rhs) {
    return FRC::Vector2(lhs.x / rhs, lhs.y / rhs);
}
FRC::Vector2& FRC::Vector2::operator/=(const FRC::Vector2& rhs) {
    x /= rhs.x;
    y /= rhs.y;
    return *this;
}
FRC::Vector2& FRC::Vector2::operator/=(const float& rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
}
FRC::Vector2& FRC::Vector2::operator/=(const int& rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
}

void initVector2(py::module &m) {
    // Vector2
    py::class_<FRC::Vector2>(m, "Vector2")
        .def(py::init<>())
        .def(py::init<float, float>(), py::arg("x"), py::arg("y"))
        .def_readwrite("x", &FRC::Vector2::x)
        .def_readwrite("y", &FRC::Vector2::y);
}