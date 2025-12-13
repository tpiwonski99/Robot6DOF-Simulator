#pragma once
#include <cmath>
#include <array>
#include <stdexcept>
#include <iostream>
#include <vector>
#include "Vector3.hpp"
#include "Matrix3.hpp"

class Matrix4 {

private:
	std::array<std::array<double, 4>, 4> elements_;

public:

	Matrix4();
	Matrix4(double m00, double m01, double m02, double m03,
		double m10, double m11, double m12, double m13,
		double m20, double m21, double m22, double m23,
		double m30, double m31, double m32, double m33);

    static Matrix4 identity();

    Matrix4(const Matrix3& R, const Vector3& t);

    double& operator()(size_t i, size_t j) { return elements_[i][j]; }
    const double& operator()(size_t i, size_t j) const { return elements_[i][j]; }
    std::array<double, 4>& operator[](size_t i);
    const std::array<double, 4>& operator[](size_t i) const;

    Matrix4 operator*(const Matrix4& other) const;
    bool operator==(const Matrix4& other) const;

    Vector3 transformPoint(const Vector3& p) const;
    Vector3 transformVector(const Vector3& v) const;

    Matrix3 rotation() const;
    Vector3 translation() const;
    Matrix4 inverseRigid() const;

    Matrix4 forwardKinematics(const std::vector<double>& q) const;

};