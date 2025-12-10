#include "Vector3.hpp"
#include "Matrix.hpp"
#include "Matrix3.hpp"
#include "Matrix4.hpp"

int main()
{
	Vector3 a(1, 2, 3);
	Vector3 b(1, 2, 3);

	Vector3 c = a + b;

	c = c * 5;

	double length = c.length();

	std::cout << c.getX() << " , " << c.getY() << " , " << c.getZ() << "\n";

	std::cout << length << "\n";

	Matrix A(3, 3);
	A[0] = { 1,2,3 };
	A[1] = { 0,1,4 };
	A[2] = { 5,6,0 };

	Matrix inv = A.inverse();

	Matrix I = A * inv;
	I.print();

	return -1;
}

/* TO DO:

Matrix4 forwardKinematics(const std::vector<double>& q) const;

*/

