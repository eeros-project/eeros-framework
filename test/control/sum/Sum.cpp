#include "SumBlockTest.hpp"
#include <eeros/math/Matrix.hpp>

void illegalNofArgument(int a) {
	std::cout << "Illegal number of arguments (" << a << ")." << std::endl;
}

void illegalDimension(int n, int m) {
	std::cout << "Illegal matrix dimension (" << n << "x" << m << "), available tests: 4x1, 2x2." << std::endl;
}

void illegalNofInputs(int i) {
	std::cout << "Illegal number of inputs (" << i << "), available tests: 2, 3 or 10 inputs." << std::endl;
}

int main(int argc, char* argv[]) {
	SumBlockTestScalar<2, double> tester2;
	SumBlockTestScalar<3, double> tester3;
	SumBlockTestScalar<10, double> tester10;
	SumBlockTestMatrix<2, 4, 1, eeros::math::Matrix<4, 1, double>> tester4x1;
	SumBlockTestMatrix<2, 2, 2, eeros::math::Matrix<2, 2, double>> tester2x2;
	
	if(argc == 3 || argc == 5) {
		switch(atoi(argv[1])) {
			case 2:
				if(argc == 5) {
					if(atoi(argv[2]) == 2 && atoi(argv[3]) == 2) {
						return tester2x2.run(argv[4]);
					}
					else if(atoi(argv[2]) == 4 && atoi(argv[3]) == 1) {
						return tester4x1.run(argv[4]);
					}
					else {
						illegalDimension(atoi(argv[2]), atoi(argv[3]));
						return -4;
					}
				}
				else {
					return tester2.run(argv[2]);
				}
				break;
			case 3:
				return tester3.run(argv[2]);
				break;
			case 10:
				return tester10.run(argv[2]);
				break;
			default:
				illegalNofInputs(atoi(argv[1]));
				return -3;
		}
	}
	else {
		illegalNofArgument(argc);
		return -2;
	}
	return -1;
}
