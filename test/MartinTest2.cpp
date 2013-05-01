#include <iostream>
#include <ostream>

#include <eeros/core/Executor.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/BlockOutput.hpp>

#define TIMETOWAIT 1

int main()
{	
	std::cout << "Test 2 started..." << std::endl;
	
	Executor e(0.01); // 10 ms period time
	
	AnSignal sig1("s1", "m");
	AnSignal sig2("s2", "m");
	
	Step step(sig1, 1, 5, 0.5);
	Gain gain(sig2, 10);
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