#pragma once
#include <iostream>
#include <cmath>

class Vector3 {

private:

	double x_, y_, z_;

public:

	Vector3(double x, double y, double z);
	~Vector3() = default;

	double getX() const;
	double getY() const;
	double getZ() const;

	void setX(double val);
	void setY(double val);
	void setZ(double val);

	Vector3 operator+ (const Vector3& other) const;
	Vector3 operator- (const Vector3& other) const;
	Vector3 operator* (double scalar) const;

	double length() const;
	double dotProduct(const Vector3& other) const;
	Vector3 cross(const Vector3& other) const;
};