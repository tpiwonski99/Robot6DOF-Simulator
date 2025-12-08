#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <iomanip>
#include "Vector3.hpp"

class Matrix {

private: 

	size_t rows_, cols_;

	std::vector<std::vector<double>> elements_;

public:

	Matrix(size_t rows, size_t cols);

	void print(int precision = 4) const;

	size_t getRows() const;
	size_t getCols() const;
	bool is_square() const;

	std::vector<double>& operator[](size_t i);
	const std::vector<double>& operator[](size_t i) const;

	Matrix operator+ (const Matrix& other) const;
	Matrix operator- (const Matrix& other) const;
	Matrix operator* (const Matrix& other) const;
	Matrix operator* (double s) const;
	Vector3 operator*(const Vector3& v) const;

	bool operator!=(const Matrix& other) const;
	bool operator== (const Matrix& other) const;

	Matrix transpose() const;
	Matrix identity() const;
	Matrix submatrix(size_t removeRow, size_t removeCol) const;
	double determinant() const;

	Matrix cofactorMatrix() const;
	Matrix inverse() const;

	bool isIdentity(double eps) const;
	static Matrix identity(size_t n);

	double trace() const;
	bool isOrthogonal(double eps) const;
};