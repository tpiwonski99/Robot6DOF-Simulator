#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>

/* TO DO
	
	DETERMINIANT
	INVERSE
	IS_SQUARE
	IS_IDENTITY

	NOWA KLASA MATRIX4: 

	FUNKCJE OBROTU - ROT X/Y/Z 
	TRANSLATION (X, Y, Z)

	ZMIANA TCP - MATRIX4 *VECTOR3 

	FOWARD KINEMATICS 
	ITD.

*/
class Matrix {

private: 

	size_t rows_, cols_;

	std::vector<std::vector<double>> elements_;

public:

	Matrix(size_t rows, size_t cols);

	size_t getRows() const;
	size_t getCols() const;

	std::vector<double>& operator[](size_t i);
	const std::vector<double>& operator[](size_t i) const;

	Matrix operator+ (const Matrix& other) const;
	Matrix operator- (const Matrix& other) const;
	Matrix operator* (const Matrix& other) const;

	Matrix transpose() const;
	Matrix identity() const;
};