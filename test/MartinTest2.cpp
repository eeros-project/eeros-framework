#include <iostream>
#include <ostream>

#include <eeros/core/Executor.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/BlockOutput.hpp>

#include <unistd.h>

#define TIMETOWAIT 1

int main() {
	using namespace eeros;
	using namespace eeros::control;
	
	std::cout << "Martin Test 2 started..." << std::endl;
	
	Executor e(0.01); // 10 ms period time
	
	Step step(1.0, 5.0, 0);
	Gain gain(10);
	BlockOutput output;
	gain.getIn().connect(step.getOut());
	output.getIn().connect(gain.getOut());
	
	e.addRunnable(&step);
	e.addRunnable(&gain);
	e.addRunnable(&output);
	e.start();
	std::cout << "waiting for " << TIMETOWAIT << " seconds while executor is running" << std::endl;
#if defined(WINDOWS)
	Sleep(TIMETOWAIT * 1000);
#else
	sleep(TIMETOWAIT);
#endif
	e.stop();
	std::cout << "waiting for executor to terminate..." << std::endl;
	while(!e.isTerminated());
	std::cout << "step output value = " << step.getOut().getValue() << std::endl;
	std::cout << "gain output value = " << gain.getOut().getValue() << std::endl;
	std::cout << "output value = " << output.getIn().getValue() << std::endl;
	std::cout << "Test 2 done..." << std::endl;
}