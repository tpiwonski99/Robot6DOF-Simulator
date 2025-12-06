#include "Matrix.hpp"

Matrix::Matrix(size_t rows, size_t cols) : rows_(rows), cols_(cols) {

	if (rows_ == 0 || cols_ == 0)
		throw std::invalid_argument("[Matrix] rows and cols must be > 0.");

	elements_.resize(rows_);

	for (auto& elem : elements_)
	{
		elem.assign(cols_, 0);
	}
}

size_t Matrix::getRows() const { return rows_; }

size_t Matrix::getCols() const { return cols_; }

std::vector<double>& Matrix::operator[](size_t i) { return elements_[i]; }

const std::vector<double>& Matrix::operator[](size_t i) const { return elements_[i]; }

Matrix Matrix::operator+ (const Matrix& other) const {

	if (rows_ != other.rows_ || cols_ != other.cols_)
		throw std::runtime_error("[Matrix] Cannot add matrices of different dimensions.");

	Matrix result(rows_, cols_);

	for (size_t r = 0; r < rows_; r++)
		for (size_t c = 0; c < cols_; c++)
			result[r][c] = elements_[r][c] + other.elements_[r][c];

	return result;
}

Matrix Matrix::operator- (const Matrix& other) const {

	if (rows_ != other.rows_ || cols_ != other.cols_)
		throw std::runtime_error("[Matrix] Cannot subtract matrices of different dimensions.");
	
	Matrix result(rows_, cols_);

	for (size_t r = 0; r < rows_; r++)
		for (size_t c = 0; c < cols_; c++)
			result[r][c] = elements_[r][c] - other.elements_[r][c];

	return result;
}

Matrix Matrix::operator* (const Matrix& other) const {

	if (cols_ != other.rows_)
		throw std::runtime_error("[Matrix] Cannot multiply matrices with different numbers of columns and rows.");

	Matrix result(rows_, other.cols_);

	for (size_t i = 0; i < rows_; i++)
		for (size_t j = 0; j < other.cols_; j++)
			for (size_t k = 0; k < cols_; k++)
				result[i][j] += elements_[i][k] * other.elements_[k][j];

	return result;
}

Matrix Matrix::transpose() const {

	Matrix result(cols_, rows_);

	for (size_t i = 0; i < rows_; i++)
		for (size_t j = 0; j < cols_; j++)
			result[j][i] = elements_[i][j];

	return result;
}

Matrix Matrix::identity() const {
	
	if (rows_ != cols_)
		throw std::runtime_error("[Matrix] Identity exists only for square matrix.");

	Matrix result(rows_, cols_);

	for (size_t i = 0; i < rows_; i++)
		for (size_t j = 0; j < cols_; j++) {
			result[i][j] = (i == j ? 1.0 : 0.0);
		}

	return result;
}

Matrix Matrix::submatrix(size_t removeRow, size_t removeCol) const {

	if (removeRow > rows_ || removeCol > cols_)
		throw std::runtime_error("[Matrix] Invalid removeRow / removeCol value.");

	Matrix result(rows_ - 1, cols_ - 1);

	size_t r = 0;
	for (size_t i = 0; i < rows_; i++) {
		if (i == removeRow) continue;

		size_t c = 0;
		for (size_t j = 0; j < cols_; j++) {
			if (j == removeCol) continue;

			result[r][c] = elements_[i][j];
			c++;
		}
		r++;
	}

	return result;
}

double Matrix::determinant() const {

	if (rows_ != cols_)
		throw std::runtime_error("[Matrix] Determinant exists only for square matrix.");

	if (rows_ == 1) return elements_[0][0];

	if (rows_ == 2)
		return (elements_[0][0] * elements_[1][1] - elements_[0][1] * elements_[1][0]);

	double result = 0;

	for (size_t j = 0; j < cols_; j++) {
		if (elements_[0][j] == 0) continue;

		Matrix minor = submatrix(0, j);

		double sign = (j % 2 == 0 ? 1 : -1);

		result += sign * elements_[0][j] * minor.determinant();
	}

	return result;
}