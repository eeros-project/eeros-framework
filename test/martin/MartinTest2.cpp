#include <iostream>
#include <ostream>

#include <eeros/core/Executor.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Gain.hpp>

#include <unistd.h>

#define TIMETOWAIT 2

int main() {
	using namespace eeros;
	using namespace eeros::control;
	
	std::cout << "Martin Test 2 started..." << std::endl;
	
	Executor e(0.01); // 10 ms period time
	
	Step<> step(1.0, 5.0, 1);
	Gain<> gain(10);
	gain.getIn().connect(step.getOut());
	
	e.addRunnable(&step);
	e.addRunnable(&gain);
	e.start();
	std::cout << "Waiting for " << TIMETOWAIT << " seconds while executor is running" << std::endl;
#if defined(WINDOWS)
	Sleep(TIMETOWAIT * 1000);
#else
	sleep(TIMETOWAIT);
#endif
	e.stop();
	std::cout << "Waiting for executor to terminate..." << std::endl;
	while(!e.isTerminated());
	std::cout << "Step output value = " << step.getOut().getSignal().getValue() << std::endl;
	std::cout << "Gain output value = " << gain.getOut().getSignal().getValue() << std::endl;
	std::cout << "Test 2 done..." << std::endl;
}