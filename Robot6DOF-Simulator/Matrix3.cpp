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

Matrix3 Matrix3::operator*(const Matrix3& other) const {
	Matrix3 result;
	
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			for (size_t k = 0; k < 3; k++)
				result[i][j] += elements_[i][k] * other.elements_[k][j];
	
	return result;
}

Vector3 Matrix3::operator*(const Vector3& v) const {
	return Vector3(
		elements_[0][0] * v.getX() + elements_[0][1] * v.getY() + elements_[0][2] * v.getZ(),
		elements_[1][0] * v.getX() + elements_[1][1] * v.getY() + elements_[1][2] * v.getZ(),
		elements_[2][0] * v.getX() + elements_[2][1] * v.getY() + elements_[2][2] * v.getZ()
	);
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
	Matrix3 shouldBeIdentity = (*this) * Rt;

	double det = this->determinant();

	return std::abs(det - 1.0) < eps;
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

	X = X * (1.0 / X.length());

	double projXY = X.dotProduct(Y);
	Y = Y - (X * projXY);
	Y = Y * (1.0 / Y.length());

	Z = X.cross(Y);

	return Matrix3(
		X.getX(), Y.getX(), Z.getX(),
		X.getY(), Y.getY(), Z.getY(),
		X.getZ(), Y.getZ(), Z.getZ());
}

Matrix3 Matrix3::fromEuler(double roll, double pitch, double yaw) {
	Matrix3 Rx = rotateX(roll);
	Matrix3 Ry = rotateY(pitch);
	Matrix3 Rz = rotateZ(yaw);

	return Rx * Ry * Rz;
}

std::tuple<double, double, double> Matrix3::toEuler() const {

	double yaw, roll;

	double cp = std::sqrt(elements_[0][0] * elements_[0][0] + elements_[1][0] * elements_[1][0]);

	double pitch = std::atan2(-elements_[2][0], cp);

	if (cp < 1e-6) {
		yaw = std::atan2(-elements_[0][1], elements_[1][1]);
		roll = 0.0;
	}
	else {
		yaw = std::atan2(elements_[1][0], elements_[0][0]);
		roll = std::atan2(elements_[2][1], elements_[2][2]);
	}

	return { roll, pitch, yaw };
}
