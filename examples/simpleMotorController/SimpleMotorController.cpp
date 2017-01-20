#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>

#include <eeros/hal/HAL.hpp>
// #include <eeros/hal/ComediFqd.hpp>
// #include <eeros/hal/ComediDac.hpp>
#include <eeros/safety/SafetySystem.hpp>
// #include <eeros/hal/ComediDigIn.hpp>
// #include <eeros/hal/ComediDigOut.hpp>
// #include <eeros/hal/ComediFqd.hpp>
// #include <eeros/hal/ComediDac.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include "MySafetyProperties.hpp"
#include "MyControlSystem.hpp"
#include "SequenceA.hpp"
#include <eeros/core/Executor.hpp>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::sequencer;

// void initHardware() {
// 	HAL& hal = HAL::instance();
// 
// 	std::cout << "  Creating device structure..." << std::endl;
// 	ComediDevice* comedi0 = new ComediDevice("/dev/comedi0");
// 
// 	std::cout << "  Registering I/Os in the HAL..." << std::endl;
// 	hal.addInput(new ComediFqd("q", comedi0, 11, 8, 10, 9, 6.28318530718 / (4 * 500.0), 0, 0));
// 	hal.addInput(new ComediDigIn("emergency", comedi0, 2, 1, true));
// 	hal.addOutput(new ComediDigOut("enable", comedi0, 2, 0));
// 	hal.addOutput(new ComediDac("dac", comedi0, 1, 0));
// }


int main() {
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show();
	
	log.info() << "Simple Motor Controller Demo started...";
	
	log.info() << "Initializing Hardware...";
// 	initHardware();
	HAL& hal = HAL::instance();
	hal.readConfigFromFile("/opt/hal/config/HalSimpleMotorControllerComedi.json");
	
	// Create the control system
	MyControlSystem controlSys(0.001);
	
	// Create and initialize a safety system
	MySafetyProperties properties(controlSys);
	SafetySystem safetySys(properties, 0.001);
	
	Sequencer sequencer;
	SequenceA mainSequence("Main Sequence", sequencer, safetySys, properties, controlSys, 3.14/5);
	sequencer.start(&mainSequence);
	
	auto &executor = Executor::instance();
	executor.setMainTask(safetySys);
	
	executor.run();
	
	sleep(100);

	safetySys.run();
	sequencer.shutdown();
	sleep(3);
	if(sequencer.getState()!=state::terminated) 
		sequencer.abort();
	
	std::cout << "Example finished..." << std::endl;
	sleep(5);
	return 0;
}
