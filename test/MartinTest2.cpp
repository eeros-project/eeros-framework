#include <iostream>
#include <ostream>

#include <unistd.h>
#include <core/Executor.hpp>

#define TIMETOWAIT 10

int test = 0;

void run() {
	test++;
}

int main()
{
	std::cout << "Test 2 started..." << std::endl;
	Executor e(0.01); // 10 ms period time
	e.addRunMethod(run);
	e.start();
	std::cout << "waiting for " << TIMETOWAIT << " seconds while executor is counting" << std::endl;
	sleep(TIMETOWAIT);
	std::cout << "test = " << test << std::endl;
	std::cout << "Test 2 done..." << std::endl;
}