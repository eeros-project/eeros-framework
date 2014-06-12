#include <iostream>
#include <cmath>
#include <eeros/math/CoordinateSystem.hpp>

using namespace eeros::math;

int main(int argc, char *argv[]) {
	int error = 0, errorSum = 0;
	int testNo = 1;
	
	std::cout << "Testing class CoordinateSystem" << std::endl;
	
	/**************************************************/
	
	std::cout << "Test #" << testNo++ << ": Creation" << std::endl;
	error = 0;
	
	CoordinateSystem a("a");
	CoordinateSystem b("b");
	bool e = false;
	try {
		CoordinateSystem b2("b");
	}
	catch(...) {
		e = true;
	}
	if(!e) {
		std::cout << "  -> Failure: creation of a second coordinate system with id 'b' should throw an exception!" << std::endl;
		error++;
	}
	
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	/**************************************************/
	
	std::cout << "Test #" << testNo++ << ": Comparision" << std::endl;
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
	
	/**************************************************/
	
	std::cout << "Test #" << testNo++ << ": get by id" << std::endl;
	error = 0;
	
	CoordinateSystem* ap = CoordinateSystem::getCoordinateSystem("a");
	CoordinateSystem* bp = CoordinateSystem::getCoordinateSystem("b");
	
	if(ap != &a) {
		std::cout << "  -> Failure: getting the coordinate system by id ('a') failed" << std::endl;
		error++;
	}
	
	if(bp != &b) {
		std::cout << "  -> Failure: getting the coordinate system by id ('b') failed" << std::endl;
		error++;
	}
	
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	/**************************************************/
	
	if(errorSum == 0) {
		std::cout << "Matrix element access test succeeded" << std::endl;
	}
	else {
		std::cout << "Matrix element access test failed with " << errorSum << " error(s)" << std::endl;
	}
	
	return errorSum;
}
