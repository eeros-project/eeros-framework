#include <iostream>
#include <cmath>
#include <eeros/math/CoordinateSystem.hpp>
#include <eeros/math/Frame.hpp>

using namespace eeros::math;

int main(int argc, char *argv[]) {
	int error = 0, errorSum = 0;
	int testNo = 1;
	
	std::cout << "Testing class Frame" << std::endl;
	
	/**************************************************/
	
	std::cout << "Test #" << testNo++ << ": Creation" << std::endl;
	error = 0;
	
	CoordinateSystem a("a");
	CoordinateSystem b("b");
	CoordinateSystem c("c");
	
	Frame ab(a, b);
	Frame bc(b, c);
	Frame ac(a, c);
	
	std::cout << "  -> Info: " << Frame::getNofFrames() << " frames created." << std::endl;
	
	bool e = false;
	try {
		Frame ab2(a, b);
	}
	catch(...) {
		e = true;
		std::cout << "  -> Info: exception catched!" << std::endl;
	}
	if(!e) {
		std::cout << "  -> Failure: creation of a second frame 'a -> b' should throw an exception!" << std::endl;
		error++;
	}
	
	errorSum += error;
	std::cout << "  -> Test finished with " << error << " error(s)" << std::endl;
	
	/**************************************************/
	
	std::cout << "Test #" << testNo++ << ": get frame by coordinate systems" << std::endl;
	error = 0;
	
	Frame* abp = Frame::getFrame(a, b);
	Frame* bcp = Frame::getFrame(b, c);
	Frame* acp = Frame::getFrame(a, c);
	
	if(abp != &ab) {
		std::cout << "  -> Failure: getting the frame 'a -> b' failed" << std::endl;
		error++;
	}
	
	if(bcp != &bc) {
		std::cout << "  -> Failure: getting the frame 'b -> c' failed" << std::endl;
		error++;
	}
	
	if(acp != &ac) {
		std::cout << "  -> Failure: getting the frame 'a -> c' failed" << std::endl;
		error++;
	}
	
	errorSum += error;
	std::cout << "  -> Test finished with " << error << " error(s)" << std::endl;
		
	/**************************************************/
	
	std::cout << "Test #" << testNo++ << ": get frame by coordinate system ids" << std::endl;
	error = 0;
	
	Frame* abps = Frame::getFrame(*CoordinateSystem::getCoordinateSystem("a"), *CoordinateSystem::getCoordinateSystem("b"));
	Frame* bcps = Frame::getFrame(*CoordinateSystem::getCoordinateSystem("b"), *CoordinateSystem::getCoordinateSystem("c"));
	Frame* acps = Frame::getFrame(*CoordinateSystem::getCoordinateSystem("a"), *CoordinateSystem::getCoordinateSystem("c"));
	
	if(abps != &ab) {
		std::cout << "  -> Failure: getting the frame 'a -> b' failed" << std::endl;
		error++;
	}
	
	if(bcps != &bc) {
		std::cout << "  -> Failure: getting the frame 'b -> c' failed" << std::endl;
		error++;
	}
	
	if(acps != &ac) {
		std::cout << "  -> Failure: getting the frame 'a -> c' failed" << std::endl;
		error++;
	}
	
	errorSum += error;
	std::cout << "  -> Test finished with " << error << " error(s)" << std::endl;
	
	/**************************************************/
	
	std::cout << "Test #" << testNo++ << ": get coordinate systems from frame" << std::endl;
	error = 0;
	
	const CoordinateSystem& aba = ab.getFromCoordinateSystem();
	const CoordinateSystem& abb = ab.getToCoordinateSystem();
	const CoordinateSystem& bcb = bc.getFromCoordinateSystem();
	const CoordinateSystem& bcc = bc.getToCoordinateSystem();
	const CoordinateSystem& aca = ac.getFromCoordinateSystem();
	const CoordinateSystem& acc = ac.getToCoordinateSystem();
	
	if(aba != a) {
		std::cout << "  -> Failure: getting from coordinate system of frame 'a -> b'" << std::endl;
		error++;
	}
	
	if(abb != b) {
		std::cout << "  -> Failure: getting to coordinate system of frame 'a -> b'" << std::endl;
		error++;
	}
	
	if(bcb != b) {
		std::cout << "  -> Failure: getting from coordinate system of frame 'b -> c'" << std::endl;
		error++;
	}
	
	if(bcc != c) {
		std::cout << "  -> Failure: getting to coordinate system of frame 'b -> c'" << std::endl;
		error++;
	}
	
	if(aca != a) {
		std::cout << "  -> Failure: getting from coordinate system of frame 'a -> c'" << std::endl;
		error++;
	}
	
	if(acc != c) {
		std::cout << "  -> Failure: getting to coordinate system of frame 'a -> c'" << std::endl;
		error++;
	}
	
	errorSum += error;
	std::cout << "  -> Test finished with " << error << " error(s)" << std::endl;
	
	/**************************************************/
	
	if(errorSum == 0) {
		std::cout << "Frame test succeeded" << std::endl;
	}
	else {
		std::cout << "Frame test failed with " << errorSum << " error(s)" << std::endl;
	}
	
	return errorSum;
}
