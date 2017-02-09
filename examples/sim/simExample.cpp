#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include "control/SimControlSystem.hpp"
#include "safety/SimSafetyProperties.hpp"
#include "sequences/SimMainSequence.hpp"
#include <iostream>

#include <signal.h>
#include <unistd.h>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::sequencer;

volatile bool running = true;
const double dt = 0.001;

void signalHandler(int signum){
	running = false;
}

int main(int argc, char **argv){
	// Create and initialize logger
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	w.show();
	
	log.info() << "Sim Example started...";
  
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);
	
	SimControlSystem* simCtrlSys; 
	simCtrlSys = new SimControlSystem(dt);
	
	// Create safety system
	SimSafetyProperties safetyProperties(simCtrlSys);
	SafetySystem safetySystem(safetyProperties, dt);
	
	// Sequencer
	Sequencer sequencer;
	SimMainSequence mainSequence(&sequencer, simCtrlSys, &safetySystem);
	sequencer.start(&mainSequence);
	
	// Set executor & create safety system
	auto &executor = Executor::instance();
	executor.setMainTask(safetySystem);
	
	// Start control system
	executor.run();
	
	while(sequencer.getState()!=state::terminated) {
		sequencer.shutdown();
		sleep(3);
	}
	
	sequencer.abort();
	
	log.info() << "Sim Example finished...";
		
	return 0;
}
