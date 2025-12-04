#include "Vector3.hpp"

Vector3::Vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}

double Vector3::getX() const { return x_; };
double Vector3::getY() const { return y_; };
double Vector3::getZ() const { return z_; };

void Vector3::setX(double val) { x_ = val; };
void Vector3::setY(double val) { y_ = val; };
void Vector3::setZ(double val) { z_ = val; };

Vector3 Vector3::operator+ (const Vector3& other) const {
	return Vector3(x_ + other.x_, y_ + other.y_, z_ + other.z_);
}

Vector3 Vector3::operator- (const Vector3& other) const {
	return Vector3(x_ - other.x_, y_ - other.y_, z_ - other.z_);
}

Vector3 Vector3::operator* (double scalar) const {
	return Vector3(x_ * scalar, y_ * scalar, z_ * scalar);
}

double Vector3::length() const {
	return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}

double Vector3::dotProduct(const Vector3& other) const {
	return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
}

Vector3 Vector3::cross(const Vector3& other) const
{
	return Vector3(
		y_ * other.z_ - other.y_ * z_,
		z_ * other.x_ - other.z_ * x_,
		x_ * other.y_ - other.x_ * y_);
}
