#include <iostream>
#include <ostream>

#include <unistd.h>
#include <core/Executor.hpp>

#define TIMETOWAIT 3

int test = 0;

void run() {
	test++;
}

int main()
{
	std::cout << "Test 1 started..." << std::endl;
	Executor e(0.1); // 100 ms period time
	e.addRunMethod(run);
	e.start();
	std::cout << "waiting for " << TIMETOWAIT << " seconds while executor is counting" << std::endl;
	sleep(TIMETOWAIT);
	e.stop();
	std::cout << "waiting for executor to terminate..." << std::endl;
	while(!e.isTerminated());
	std::cout << "test = " << test << std::endl;
	std::cout << "Test 1 done..." << std::endl;
}