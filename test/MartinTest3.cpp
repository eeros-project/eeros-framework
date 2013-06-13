#include <iostream>
#include <ostream>
#include <fstream>

#include <eeros/core/Executor.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/BlockOutput.hpp>

#define TIMETOWAIT 1

int main()
{	
	std::cout << "Martin Test 3 started..." << std::endl;
	
	Executor e(0.01); // 10 ms period time
		
	Step step(1, 5, 0.5);
	Gain gain(10);
	BlockOutput output;
	gain.in.connect(step.out);
	output.in.connect(gain.out);
	
	e.addRunnable(&step);
	e.addRunnable(&gain);
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