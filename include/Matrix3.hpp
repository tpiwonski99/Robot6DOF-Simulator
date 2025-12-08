#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>  
#include <array>
#include <tuple>
#include <iomanip>
#include "Vector3.hpp"

class Matrix3 {

private:
    std::array<std::array<double, 3>, 3> elements_;

public:

    Matrix3();
    Matrix3(double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22);

    ~Matrix3() = default;

    void print(int precision = 3) const;

    double& operator()(size_t i, size_t j) { return elements_[i][j]; }
    const double& operator()(size_t i, size_t j) const { return elements_[i][j]; }
    std::array<double, 3>& operator[](size_t i) { return elements_[i]; }
    const std::array<double, 3>& operator[](size_t i) const { return elements_[i]; }

    Matrix3 operator*(const Matrix3& other) const;
    bool operator==(const Matrix3& other) const;
    Vector3 operator*(const Vector3& v) const;

    Matrix3 transpose() const;

    static Matrix3 rotateX(double angle);
    static Matrix3 rotateY(double angle);
    static Matrix3 rotateZ(double angle);

    bool isOrthogonal(double eps = 1e-9) const;
    double determinant() const;
    double trace() const;
    Matrix3 orthonormalize() const;

    static Matrix3 fromEuler(double roll, double pitch, double yaw);
    Vector3 toEuler() const;

    static Matrix3 fromAxisAngle(const Vector3& axis, double angle);
    void toAxisAngle(Vector3& axis, double& angle) const;
};