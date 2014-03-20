#include "SumBlockTest.hpp"

int main(int argc, char* argv[]) {
	SumBlockTest<2> tester2;
	SumBlockTest<3> tester3;
	SumBlockTest<10> tester10;
	if (argc == 3) {
		switch(atoi(argv[1])) {
			case 2:
				return tester2.run(argv[2]);
				break;
			case 3:
				return tester3.run(argv[2]);
				break;
			case 10:
				return tester10.run(argv[2]);
				break;
			default:
				std::cout << "illegal number of inputs" << std::endl;
				return -4;
		}
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}
