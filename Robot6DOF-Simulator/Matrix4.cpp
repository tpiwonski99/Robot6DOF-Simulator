#include "Matrix4.hpp"

Matrix4::Matrix4() : elements_{ { {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1} } } {}

Matrix4::Matrix4(double m00, double m01, double m02, double m03,
	double m10, double m11, double m12, double m13,
	double m20, double m21, double m22, double m23,
	double m30, double m31, double m32, double m33) 
{
	elements_[0][0] = m00;  elements_[0][1] = m01;  elements_[0][2] = m02, elements_[0][3] = m03;
	elements_[1][0] = m10;  elements_[1][1] = m11;  elements_[1][2] = m12, elements_[1][3] = m13;
	elements_[2][0] = m20;  elements_[2][1] = m21;  elements_[2][2] = m22, elements_[2][3] = m23;
	elements_[3][0] = m30;  elements_[3][1] = m31;  elements_[3][2] = m32, elements_[3][3] = m33;
}

Matrix4::Matrix4(const Matrix3& R, const Vector3& t) {
	elements_[0][0] = R[0][0],  elements_[0][1] = R[0][1];  elements_[0][2] = R[0][2], elements_[0][3] = t.getX();
	elements_[1][0] = R[1][0],	elements_[0][1] = R[1][1];  elements_[1][2] = R[1][2], elements_[1][3] = t.getY();
	elements_[2][0] = R[2][0],	elements_[2][1] = R[2][1];  elements_[2][2] = R[2][2], elements_[2][3] = t.getZ();
	elements_[3][0] = 0.0;		elements_[3][1] = 0.0;		elements_[3][2] = 0.0,	   elements_[3][3] = 1.0;
}

Matrix4 Matrix4::identity() {
	
	Matrix4 result;

	return result;
}

std::array<double, 4>& Matrix4::operator[](size_t i) { return elements_[i]; }
const std::array<double, 4>& Matrix4::operator[](size_t i) const { return elements_[i]; }

Matrix4 Matrix4::operator*(const Matrix4& other) const {  
    Matrix4 result;  

    for (size_t i = 0; i < 4; i++)  
        for (size_t j = 0; j < 4; j++)  
            for (size_t k = 0; k < 4; k++)  
                result[i][j] += elements_[i][k] * other.elements_[k][j];  

    return result;  
}
bool Matrix4::operator==(const Matrix4& other) const {
	const double EPS = 1e-9;

	for (size_t i = 0; i < 4; i++)
		for (size_t j = 0; j < 4; j++)
			if (std::abs(elements_[i][j] - other.elements_[i][j]) > EPS)
				return false;

	return true;
}