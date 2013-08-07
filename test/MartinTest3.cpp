#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>

#include <eeros/core/Executor.hpp>
#include <eeros/core/SharedMemory.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/BlockOutput.hpp>
#include <eeros/control/GlobalScope.hpp>
#include <eeros/control/SignalBufferReader.hpp>

#define TIMETOWAIT 20

class Reader : public Runnable {
public:
	Reader(void* memory, uint32_t size) : r(memory, size) {}
	
	void run() {
		if(r.signalTypeAvailableToRead() == kSignalTypeReal) {
			r.readRealSignal(&id, &ts, &val);
			std::cout << '#' << id << ' ' << ts << ':' << val << std::endl;
		}
	}

private:
	SignalBufferReader r;
	sigid_t id;
	uint64_t ts;
	double val;
};

int main() {
	std::cout << "Martin Test 3 started..." << std::endl;
	
	std::cout << "Creating executors..." << std::endl;
	Executor e1(0.1); // 100 ms period time
	Executor e2(0.01); // 10 ms period time
	
	std::cout << "Creating and connecting control system elements..." << std::endl;
	Step step(1.0, 5.0, 1.0);
	step.getOut().setName("M");
	step.getOut().setUnit("Nm");
	step.getOut().setCoordinateSystem("ddx");
	
	Gain gain(10);
	gain.getOut().setName("I");
	gain.getOut().setUnit("A");
	gain.getOut().setCoordinateSystem("ddy");
	
	BlockOutput output;
	GlobalScope globalScope;
	
	gain.getIn().connect(step.getOut());
	output.getIn().connect(gain.getOut());
	
	std::cout << "Available signals:" << std::endl;
	for(std::list<Signal*>::iterator i = Signal::getSignalList()->begin(); i != Signal::getSignalList()->end(); i++) {
		uint32_t length = (*i)->getDimension();
		for(uint32_t j = 0; j < length; j++) {
			std::cout << "  " << (*i)->getLabel(j) << std::endl;
		}
	}
	
	e1.addRunnable(step);
	e1.addRunnable(gain);
	e1.addRunnable(output);
	e1.addRunnable(globalScope);
	
	std::cout << "Creating reader..." << std::endl;
	Reader r(globalScope.getSharedMemory(), kSharedMemorySize);
	e2.addRunnable(r);
	
	std::cout << "Starting executors..." << std::endl;
	e1.start();
	e2.start();
	
	std::cout << "Waiting for " << TIMETOWAIT << " seconds while executors are running" << std::endl;
	sleep(TIMETOWAIT);
 
	std::cout << "Stopping executors..." << std::endl;
	e1.stop();
	e2.stop();
	
	std::cout << "Waiting for executors to terminate..." << std::endl;
	while(!e1.isTerminated() && !e2.isTerminated());
//	while(!e1.isTerminated());
	
	std::cout << "Output value = " << output.getIn().getValue() << std::endl;
	
	std::cout << "Test 3 done..." << std::endl;
}
