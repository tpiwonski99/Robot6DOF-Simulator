#include "Vector3.hpp"
#include "Matrix.hpp"

int main()
{
	Vector3 a(1, 2, 3);
	Vector3 b(1, 2, 3);

	Vector3 c = a + b;

	c = c * 5;

	double length = c.length();

	std::cout << c.getX() << " , " << c.getY() << " , " << c.getZ() << "\n";

	std::cout << length << "\n";

	return -1;
}