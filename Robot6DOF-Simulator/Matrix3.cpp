#include "Matrix3.hpp"

Matrix3::Matrix3() : elements_{ { {1,0,0}, {0,1,0}, {0,0,1} } } {}

Matrix3::Matrix3(double m00, double m01, double m02,
	double m10, double m11, double m12,
	double m20, double m21, double m22)
{
	elements_[0][0] = m00;  elements_[0][1] = m01;  elements_[0][2] = m02;
	elements_[1][0] = m10;  elements_[1][1] = m11;  elements_[1][2] = m12;
	elements_[2][0] = m20;  elements_[2][1] = m21;  elements_[2][2] = m22;
}

void Matrix3::print(int precision) const {

	std::cout << std::fixed << std::setprecision(precision);

	for (size_t i = 0; i < 3; i++) {

		std::cout << "[ ";

		for (size_t j = 0; j < 3; j++) {
			std::cout << std::setw(8) << elements_[i][j] << " ";
		}

		std::cout << "]" << std::endl;
	}
}

Matrix3 Matrix3::operator*(const Matrix3& other) const {
	Matrix3 result;
	
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++) {
			result[i][j] = 0.0;
			for (size_t k = 0; k < 3; k++)
				result[i][j] += elements_[i][k] * other.elements_[k][j];
		}
	
	return result;
}

Vector3 Matrix3::operator*(const Vector3& v) const {
	return Vector3(
		elements_[0][0] * v.getX() + elements_[0][1] * v.getY() + elements_[0][2] * v.getZ(),
		elements_[1][0] * v.getX() + elements_[1][1] * v.getY() + elements_[1][2] * v.getZ(),
		elements_[2][0] * v.getX() + elements_[2][1] * v.getY() + elements_[2][2] * v.getZ()
	);
}

bool Matrix3::operator==(const Matrix3& other) const {
	const double EPS = 1e-9;

	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			if (std::abs(elements_[i][j] - other.elements_[i][j]) > EPS)
				return false;

	return true;
}

Matrix3 Matrix3::transpose() const {
	Matrix3 result;

	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			result[i][j] = elements_[j][i];

	return result;
}

Matrix3 Matrix3::rotateX(double angle) {
	double c = std::cos(angle);
	double s = std::sin(angle);

	return Matrix3(1, 0, 0,		0, c, -s,	0, s, c);
}

Matrix3 Matrix3::rotateY(double angle) {
	double c = std::cos(angle);
	double s = std::sin(angle);

	return Matrix3(c, 0, s,		0, 1, 0,	-s, 0, c);
}

Matrix3 Matrix3::rotateZ(double angle) {
	double c = std::cos(angle);
	double s = std::sin(angle);

	return Matrix3(c, -s, 0,	s, c, 0,	0, 0, 1);
}

bool Matrix3::isOrthogonal(double eps) const {
	Matrix3 Rt = this->transpose();
	Matrix3 I = (*this) * Rt;

	for (size_t i = 0; i < 3; ++i)
		for (size_t j = 0; j < 3; ++j) {
			double expected = (i == j) ? 1.0 : 0.0;
			if (std::fabs(I[i][j] - expected) > eps)
				return false;
		}

	double det = this->determinant();
	if (std::fabs(std::fabs(det) - 1.0) > eps)
		return false;

	return true;
}

double Matrix3::determinant() const {
	return
		elements_[0][0] * (elements_[1][1] * elements_[2][2] - elements_[1][2] * elements_[2][1])
		- elements_[0][1] * (elements_[1][0] * elements_[2][2] - elements_[1][2] * elements_[2][0])
		+ elements_[0][2] * (elements_[1][0] * elements_[2][1] - elements_[1][1] * elements_[2][0]);
}

double Matrix3::trace() const {
	return elements_[0][0] + elements_[1][1] + elements_[2][2];
}

Matrix3 Matrix3::orthonormalize() const {

	Vector3 X(elements_[0][0], elements_[1][0], elements_[2][0]);
	Vector3 Y(elements_[0][1], elements_[1][1], elements_[2][1]);
	Vector3 Z(elements_[0][2], elements_[1][2], elements_[2][2]);
	
	double lenX = X.length();
	if (lenX < 1e-12) throw std::runtime_error("Cannot orthonormalize: X is zero.");

	X = X * (1.0 / lenX);

	double projXY = X.dotProduct(Y);
	Y = Y - (X * projXY);

	double lenY = Y.length();
	if (lenY < 1e-12) throw std::runtime_error("Cannot orthonormalize: Y is collinear with X.");

	Y = Y * (1.0 / lenY);

	Z = X.cross(Y);

	return Matrix3(
		X.getX(), Y.getX(), Z.getX(),
		X.getY(), Y.getY(), Z.getY(),
		X.getZ(), Y.getZ(), Z.getZ());
}

Matrix3 Matrix3::fromEuler(double roll, double pitch, double yaw)
{
	double cr = cos(roll), sr = sin(roll);
	double cp = cos(pitch), sp = sin(pitch);
	double cy = cos(yaw), sy = sin(yaw);

	return Matrix3(
		cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
		sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
		-sp, cp * sr, cp * cr);
}

Vector3 Matrix3::toEuler() const {
	double v = elements_[2][0];
	
	if (v > 1.0) v = 1.0;
	if (v < -1.0) v = -1.0;
	
	double pitch = -asin(v);

	double roll, yaw;

	double cp = cos(pitch);

	if (fabs(cp) > 1e-6) {
		roll = atan2(elements_[2][1] / cp, elements_[2][2] / cp);
		yaw = atan2(elements_[1][0] / cp, elements_[0][0] / cp);
	}
	else {
		roll = 0.0;
		yaw = atan2(-elements_[0][1], elements_[1][1]);
	}

	return Vector3(roll, pitch, yaw);
}

Matrix3 Matrix3::fromAxisAngle(const Vector3& axis, double angle) {

	double ax = axis.getX();
	double ay = axis.getY();
	double az = axis.getZ();

	double len = std::sqrt(ax * ax + ay * ay + az * az);

	if (len < 1e-8)
		throw std::runtime_error("[Matrix3] Given vector length is equal to zero.");

	double nx = ax / len;
	double ny = ay / len;
	double nz = az / len;
	
	double c = std::cos(angle);
	double s = std::sin(angle);
	double t = 1.0 - c;

	double m00 = t * nx * nx + c;
	double m11 = t * ny * ny + c;
	double m22 = t * nz * nz + c;

	double m01 = t * nx * ny - s * nz;
	double m10 = t * nx * ny + s * nz;

	double m02 = t * nx * nz + s * ny;
	double m20 = t * nx * nz - s * ny;

	double m12 = t * ny * nz - s * nx;
	double m21 = t * ny * nz + s * nx;

	return Matrix3 (
		m00, m01, m02, 
		m10, m11, m12,
		m20, m21, m22);
}

void Matrix3::toAxisAngle(Vector3& axis, double& angle) const {
	const double eps = 1e-6;
	const double eps2 = 1e-12;

    double trace = this->trace();
	double cos_theta = (trace - 1) * 0.5;

	if (cos_theta > 1.0) cos_theta = 1.0;
	if (cos_theta < -1.0) cos_theta = -1.0;

	angle = std::acos(cos_theta);

	if (std::fabs(angle) < eps) {
		axis = Vector3(1.0, 0.0, 0.0);
		angle = 0.0;
		return;
	}

	if (M_PI - angle < eps) {
		
		double xx = (elements_[0][0] + 1.0) * 0.5;
		double yy = (elements_[1][1] + 1.0) * 0.5;
		double zz = (elements_[2][2] + 1.0) * 0.5;

		double xy = (elements_[0][1] + elements_[1][0]) * 0.25;
		double xz = (elements_[0][2] + elements_[2][0]) * 0.25;
		double yz = (elements_[1][2] + elements_[2][1]) * 0.25;

		double x, y, z;
		if (xx > yy && xx > zz) {
			x = std::sqrt(std::max(xx, 0.0));
			if (x < eps2) {
				y = 0.0;
				z = 0.0;
			}
			else {
				y = xy / x;
				z = xz / x;
			}
		}

		else if (yy > zz) {
			y = std::sqrt(std::max(yy, 0.0));
			if (y < eps2) {
				x = 0.0;
				z = 0.0;
			}
			else {
				x = xy / y;
				z = yz / y;
			}
		}
		else {
			z = std::sqrt(std::max(zz, 0.0));
			if (z < eps2) {
				x = 0.0;
				y = 0.0;
			}
			else {
				x = xz / z;
				y = yz / z;
			}
		}

		axis = Vector3(x, y, z);
		double len = axis.length();
		if (len > eps2) {
			axis = axis * (1.0 / len);
		}
		else {
			axis = Vector3(1.0, 0.0, 0.0); 
		}
		return;
	}

	double s = std::sin(angle);
	double denom = 2.0 * s;

	double nx = (elements_[2][1] - elements_[1][2]) / denom;
	double ny = (elements_[0][2] - elements_[2][0]) / denom;
	double nz = (elements_[1][0] - elements_[0][1]) / denom;

	axis = Vector3(nx, ny, nz);
	double len = axis.length();
	if (len > eps2) {
		axis = axis * (1.0 / len);
	}
	else {
		axis = Vector3(1.0, 0.0, 0.0);
	}
}
