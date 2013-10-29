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

int main() {
	std::cout << "Simple Motor Controller Demo started..." << std::endl;

	std::cout << "Creating executor..." << std::endl;
	Executor e1(0.001); // control loop -> 1 ms period time

	std::cout << "Creating control system elements..." << std::endl;
	Step step(0.0, -3.14159265359, 5);
	step.getOut().setName("phi_desired");
	step.getOut().setUnit("rad");

	ComediEncoder enc;
	enc.getOut().setName("phi_actual");
	enc.getOut().setUnit("rad");

	D diff1;
	diff1.getOut().setName("phi_d_actual");
	diff1.getOut().setUnit("rad/s");

	Sum sum1;
	sum1.negateInput(1);
	sum1.getOut().setName("phi_e");
	sum1.getOut().setUnit("rad");
	
	Gain posController(174.5); // kp=174.5
	posController.getOut().setName("phi_d_set");
	posController.getOut().setUnit("rad/s");

	D diff2;
	diff1.getOut().setName("phi_d_set_ff");
	diff1.getOut().setUnit("rad/s");
	
	Sum sum2(3);
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

	std::cout << "Available signals:" << std::endl;
	for(std::list<Signal*>::iterator i = Signal::getSignalList()->begin(); i != Signal::getSignalList()->end(); i++) {
		uint32_t length = (*i)->getDimension();
		for(uint32_t j = 0; j < length; j++) {
			std::cout << "  " << (*i)->getLabel(j) << std::endl;
		}
	}
	
	std::cout << "Connecting control system elements..." << std::endl;
	diff1.getIn().connect(enc.getOut());
	sum1.getIn(0).connect(step.getOut());
	sum1.getIn(1).connect(enc.getOut());
	posController.getIn().connect(sum1.getOut());
	sum2.getIn(0).connect(posController.getOut());
	sum2.getIn(1).connect(diff1.getOut());
	sum2.getIn(2).connect(diff2.getOut());
	speedController.getIn().connect(sum2.getOut());
	inertia.getIn().connect(speedController.getOut());
	invMotConst.getIn().connect(inertia.getOut());
	dac.getIn().connect(invMotConst.getOut());
    
	std::cout << "Adding blocks to executor..." << std::endl;
	e1.addRunnable(step);
	e1.addRunnable(enc);
	e1.addRunnable(sum1);
	e1.addRunnable(posController);
	e1.addRunnable(diff1);
	e1.addRunnable(sum2);
	e1.addRunnable(speedController);
	e1.addRunnable(inertia);
	e1.addRunnable(invMotConst);
	e1.addRunnable(dac);
	e1.addRunnable(globalSignalProvider);
	
	std::cout << "Starting executor..." << std::endl;
	e1.start();

	std::cout << "Waiting for " << TIMETOWAIT << " seconds while executor is running" << std::endl;
	sleep(TIMETOWAIT);

	std::cout << "Stopping executor..." << std::endl;
	e1.stop();

	std::cout << "Waiting for executor to terminate..." << std::endl;
 	while(!e1.isTerminated());
        
	std::cout << "Example finished..." << std::endl;
}
