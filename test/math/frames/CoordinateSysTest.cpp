#include <iostream>
#include <cmath>
#include <eeros/math/CoordinateSystem.hpp>

using namespace eeros::math;

int main(int argc, char *argv[]) {
	int error = 0, errorSum = 0;
	int testNo = 1;
	
	std::cout << "Testing class CoordinateSystem" << std::endl;
	
	CoordinateSystem a("a");
	CoordinateSystem b("b");
	
	std::cout << "Test #" << testNo++ << ": TODO" << std::endl;
	error = 0;
	
	if(a != a) {
		std::cout << "  -> Failure: != comparison failed" << std::endl;
		error++;
	}
	
	if(a == b) {
		std::cout << "  -> Failure: == comparison failed" << std::endl;
		error++;
	}
	
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	// ********** END **********
	
	if(errorSum == 0) {
		std::cout << "Matrix element access test succeeded" << std::endl;
	}
	else {
		std::cout << "Matrix element access test failed with " << errorSum << " error(s)" << std::endl;
	}
	
	return errorSum;
}
