#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <eeros/core/Executor.hpp>
#include <eeros/core/SharedMemory.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/GlobalSignalProvider.hpp>
#include <eeros/control/SignalBufferReader.hpp>

#include "ComediDac.hpp"
#include "ComediEncoder.hpp"

#define TIMETOWAIT 30

class Reader : public Runnable {
public:
	Reader(void* memory, uint32_t size) : r(memory, size) {}
	
	void run() {
		if(r.signalTypeAvailableToRead() == kSignalTypeReal) {
			r.readRealSignal(&id, &ts, &val);
			std::cout << '#' << (id >> 16) << ' ' << ts << ':' << val << std::endl;
		}
	}

private:
	SignalBufferReader r;
	sigid_t id;
	uint64_t ts;
	double val;
};

int main() {
	std::cout << "Simple Motor Controller Demo started..." << std::endl;

	std::cout << "Creating executors..." << std::endl;
	Executor e1(0.001); // control loop -> 1 ms period time
	Executor e2(0.1); // output -> 100 ms period time

	std::cout << "Creating and connecting control system elements..." << std::endl;

	Step step(0.0, -3.14159265359, 5);
	step.getOut().setName("phi_desired");
	step.getOut().setUnit("rad");

	ComediEncoder enc;
	enc.getOut().setName("phi_actual");
	enc.getOut().setUnit("rad");

	D diff;
	diff.getOut().setName("phi_d_actual");
	diff.getOut().setUnit("rad/s");

	Sum sum1;
	sum1.negateInput(1);
	sum1.getOut().setName("phi_e");
	sum1.getOut().setUnit("rad");

	Gain posController(174.5); // kp=174.5
	posController.getOut().setName("phi_d_set");
	posController.getOut().setUnit("rad/s");

	Sum sum2;
	sum2.negateInput(1);
	sum2.getOut().setName("phi_d_e");
	sum2.getOut().setUnit("rad/s");

	Gain speedController(565.48); // kv=565.48
	speedController.getOut().setName("phi_dd_set");
	speedController.getOut().setUnit("rad/s^2");

	Gain inertia(14.2e-7); // [kgm²]
	inertia.getOut().setName("M");
	inertia.getOut().setUnit("Nm");

	Gain invMotConst(1/15.7e-3 * 2.0); // [A/Nm * V/A]
	invMotConst.getOut().setName("i");
	invMotConst.getOut().setUnit("A");
		
	ComediDac dac(0);
	GlobalSignalProvider globalSignalProvider;

	diff.getIn().connect(enc.getOut());
	sum1.getIn(0).connect(step.getOut());
	sum1.getIn(1).connect(enc.getOut());
	posController.getIn().connect(sum1.getOut());
	sum2.getIn(0).connect(posController.getOut());
	sum2.getIn(1).connect(diff.getOut());
	speedController.getIn().connect(sum2.getOut());
	inertia.getIn().connect(speedController.getOut());
	invMotConst.getIn().connect(inertia.getOut());
	dac.getIn().connect(invMotConst.getOut());
    
	std::cout << "Available signals:" << std::endl;
	for(std::list<Signal*>::iterator i = Signal::getSignalList()->begin(); i != Signal::getSignalList()->end(); i++) {
		uint32_t length = (*i)->getDimension();
		for(uint32_t j = 0; j < length; j++) {
			std::cout << "  " << (*i)->getLabel(j) << std::endl;
		}
	}
    
	e1.addRunnable(step);
	e1.addRunnable(enc);
	e1.addRunnable(sum1);
	e1.addRunnable(posController);
	e1.addRunnable(diff);
	e1.addRunnable(sum2);
	e1.addRunnable(speedController);
	e1.addRunnable(inertia);
	e1.addRunnable(invMotConst);
	e1.addRunnable(dac);
	e1.addRunnable(globalSignalProvider);

	std::cout << "Creating reader..." << std::endl;
	Reader r(globalSignalProvider.getSharedMemory(), kSharedMemorySize);
	e2.addRunnable(r);
	
	std::cout << "Starting executors..." << std::endl;
	e1.start();
	e2.start();

	std::cout << "Waiting for " << TIMETOWAIT << " seconds while executors are running" << std::endl;
	sleep(TIMETOWAIT);

	std::cout << "Stopping executor..." << std::endl;
	e1.stop();
	e2.stop();

	std::cout << "Waiting for executors to terminate..." << std::endl;
	while(!e1.isTerminated() && !e2.isTerminated());
// 	while(!e1.isTerminated());
    
	std::cout << "Output value = " << dac.getIn().getValue() << std::endl;
    
	std::cout << "Example done..." << std::endl;
}
