#include "Vector3.hpp"

Vector3::Vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}

double Vector3::getX() const { return x_; }
double Vector3::getY() const { return y_; }
double Vector3::getZ() const { return z_; }

void Vector3::setX(double val) { 
	if (std::isnan(val) || std::isinf(val))
		throw std::invalid_argument("[Vector3] invalid X value.");

	x_ = val; 
};

void Vector3::setY(double val) { 
	if (std::isnan(val) || std::isinf(val))
		throw std::invalid_argument("[Vector3] invalid Y value.");

	y_ = val; 
};

void Vector3::setZ(double val) { 
	if (std::isnan(val) || std::isinf(val))
		throw std::invalid_argument("[Vector3] invalid Z value.");

	z_ = val; 
};

Vector3 Vector3::operator+ (const Vector3& other) const {
	return Vector3(x_ + other.x_, y_ + other.y_, z_ + other.z_);
}

Vector3 Vector3::operator- (const Vector3& other) const {
	return Vector3(x_ - other.x_, y_ - other.y_, z_ - other.z_);
}

Vector3 Vector3::operator* (double scalar) const {
	if (std::isnan(scalar) || std::isinf(scalar))
		throw std::invalid_argument("[Vector3] invalid scalar value.");

	return Vector3(x_ * scalar, y_ * scalar, z_ * scalar);
}

Vector3& Vector3::operator+=(const Vector3& other) {
    x_ += other.x_;
    y_ += other.y_;
    z_ += other.z_;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
    x_ -= other.x_;
    y_ -= other.y_;
    z_ -= other.z_;
    return *this;
}

Vector3& Vector3::operator*=(double scalar) {
    x_ *= scalar;
    y_ *= scalar;
    z_ *= scalar;
    return *this;
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

Vector3 Vector3::normalized() const {
    double len = length();
    if (len < 1e-12)
        throw std::runtime_error("[Vector3] Cannot normalize zero-length vector.");
    return Vector3(x_ / len, y_ / len, z_ / len);
}

void Vector3::normalize() {
    double len = length();
    if (len < 1e-12)
        throw std::runtime_error("[Vector3] Cannot normalize zero-length vector.");
    x_ /= len;
    y_ /= len;
    z_ /= len;
}

bool Vector3::empty() const {

    double eps = 1e-12;

    if (x_ < eps && y_ < eps && z_ < eps) return false;

    return true;
}