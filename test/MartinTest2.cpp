#include <iostream>
#include <ostream>

#include <unistd.h>
#include <core/Executor.hpp>

#include <control/Step.hpp>
#include <control/BlockOutput.hpp>

#define TIMETOWAIT 1

int main()
{
	std::cout << "Test 2 started..." << std::endl;
	Executor e(0.01); // 10 ms period time
	
	Step step;
	BlockOutput output;
	output.in.connect(step.out);
	
	e.addRunnable(&step);
	e.addRunnable(&output);
	e.start();
	std::cout << "waiting for " << TIMETOWAIT << " seconds while executor is running" << std::endl;
	sleep(TIMETOWAIT);
	e.stop();
	std::cout << "waiting for executor to terminate..." << std::endl;
	while(!e.isTerminated());
	std::cout << "output value = " << output.in.getSignal()->getValue() << std::endl;
	std::cout << "Test 2 done..." << std::endl;
}