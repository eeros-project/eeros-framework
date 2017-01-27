#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include "MySafetyProperties.hpp"
#include "MyControlSystem.hpp"
#include "SequenceA.hpp"

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::sequencer;

void signalHandler(int signum){
	SafetySystem::exitHandler();
}

int main() {
	signal(SIGINT, signalHandler);
	signal(SIGKILL, signalHandler);
	signal(SIGTERM, signalHandler);
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show();
	
	log.info() << "Simple Motor Controller Demo started...";
	
	log.info() << "Initializing Hardware...";
	HAL& hal = HAL::instance();
	hal.readConfigFromFile("/opt/hal/config/HalSimpleMotorControllerComedi.json");
	
	// Create the control system
	MyControlSystem controlSys(0.001);
	
	// Create and initialize a safety system
	MySafetyProperties properties(controlSys);
	SafetySystem safetySys(properties, 0.001);
	
	Sequencer sequencer;
	SequenceA mainSequence("Main Sequence", sequencer, safetySys, properties, controlSys, 3.14/10);
	sequencer.start(&mainSequence);
	
	auto &executor = Executor::instance();
	executor.setMainTask(safetySys);
	safetySys.triggerEvent(properties.doSystemOn);
	
	executor.run();
	
	
	while(sequencer.getState()!=state::terminated) {
		sequencer.shutdown();
		sleep(3);
	}
	
	sequencer.abort();
	
	log.info() << "Example finished...";
	return 0;
}
