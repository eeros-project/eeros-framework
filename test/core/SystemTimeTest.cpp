#include <eeros/core/System.hpp>

#include <time.h>

#include <iostream>

using namespace eeros;

int main(int argc, char* argv[]) {
	std::cout << "System time test started" << std::endl;
	
	int error = 0, errorSum = 0;
	int testNo = 1;
	
	// ********** TEST 1 **********
	
	std::cout << "#" << testNo++ << ": Reading system clock resolution" << std::endl;
	error = 0;
	double res;
	try {
		res = System::getClockResolution();
		std::cout << "  System clock resolution: " << res << " s" << std::endl;
	}
	catch(...) {
		std::cout << "  -> Failure: exception while reading system clock resolution!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "  -> Test finished with " << error << " error(s)" << std::endl;
	
	
	// ********** TEST 2 **********
	
	std::cout << "#" << testNo++ << ": Reading system time as timestamp in nano seconds" << std::endl;
	error = 0;
	uint64_t timestamp;
	try {
		timestamp = System::getTimeNs();
		std::cout << "  Timestamp: " << timestamp << " ns" << std::endl;
	}
	catch(...) {
		std::cout << "  -> Failure: exception while reading system clock!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "  -> Test finished with " << error << " error(s)" << std::endl;
	
	
	// ********** TEST 3 **********
	
	std::cout << "#" << testNo++ << ": Reading system time in seconds" << std::endl;
	error = 0;
	double time;
	try {
		time = System::getTime();
		std::cout << "  Time: " << time << " s" << std::endl;
	}
	catch(...) {
		std::cout << "  -> Failure: exception while reading system clock!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "  -> Test finished with " << error << " error(s)" << std::endl;
		
	// ********** END **********
	
	if(errorSum == 0) {
		std::cout << "System time test succeeded" << std::endl;
	}
	else {
		std::cout << "System time test failed with " << errorSum << " error(s)" << std::endl;
	}
	
	return errorSum;
} 
