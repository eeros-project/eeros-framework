#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>

#include <eeros/core/Executor.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ComediFqd.hpp>
#include <eeros/hal/ComediDac.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/hal/ComediDigIn.hpp>
#include <eeros/hal/ComediDigOut.hpp>
#include <eeros/hal/ComediFqd.hpp>
#include <eeros/hal/ComediDac.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include "MySafetyProperties.hpp"
#include "MyControlSystem.hpp"
#include "SequenceA.hpp"

#define TIMETOWAIT 30

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::sequencer;

void initHardware() {
	HAL& hal = HAL::instance();

	std::cout << "  Creating device structure..." << std::endl;
	ComediDevice* comedi0 = new ComediDevice("/dev/comedi0");

	std::cout << "  Registering I/Os in the HAL..." << std::endl;
	hal.addSystemInput(new ComediFqd("q", comedi0, 11, 8, 10, 9, 6.28318530718 / (4 * 500.0), 0, 0));
	hal.addSystemInput(new ComediDigIn("emergency", comedi0, 2, 1, true));
	hal.addSystemOutput(new ComediDigOut("enable", comedi0, 2, 0));
	hal.addSystemOutput(new ComediDac("dac", comedi0, 1, 0));
}


int main() {
	std::cout << "Simple Motor Controller Demo started..." << std::endl;

	StreamLogWriter w(std::cout);
	
	std::cout << "Initializing Hardware..." << std::endl;
	initHardware();
	
	// Get Safety System instance
	SafetySystem& safetySys = SafetySystem::instance();
	safetySys.log.set(w);
	
	// Initialize Safety System
	MySafetyProperties properties;
	safetySys.setProperties(properties);
	
	std::cout << "Creating executors..." << std::endl;
	Executor safetySysExecutor(1); // safety system -> 1 ms period time
	safetySysExecutor.addRunnable(safetySys);

	// Start safety system
	safetySysExecutor.start();
	
	// Get control System instance
	MyControlSystem& controlSys = MyControlSystem::instance();

	// Start control system
	controlSys.start();
	
	SequenceA mainSequence("Main Sequence", 2*3.14);
	Sequencer sequencer("Example sequencer", mainSequence);
	
	while(!sequencer.done());
	
	controlSys.stop();
	safetySysExecutor.stop();
	
	sleep(1);
	
	std::cout << "Example finished..." << std::endl;
}
