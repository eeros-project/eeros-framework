#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/JsonParser.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include "control/ParserTestControlSystem.hpp"
#include "safety/ParserTestSafetyProperties.hpp"
#include "sequences/ParserTestMainSequence.hpp"
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

int main(){
	// Create and initialize logger
	/*StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	w.show();
	
	log.info() << "ParserTest started...";*/
	std::cout << "ParserTest started..." << std::endl;
  
	HAL& hal = HAL::instance();
	hal.readConfigFromFile("/mnt/data/config/HALConfigExample.json");
	
	hal.callOutputFeature("pwm1", "setPwmFrequency", 100.0);
	
	ParserTestControlSystem* parserTestCtrlSys; 
	parserTestCtrlSys = new ParserTestControlSystem(dt);
		
	// Create safety system
	ParserTestSafetyProperties safetyProperties(parserTestCtrlSys);
	SafetySystem safetySystem(safetyProperties, dt);
	
	// Sequencer
	Sequencer sequencer;
	ParserTestMainSequence mainSequence(&sequencer, parserTestCtrlSys, &safetySystem);
	sequencer.start(&mainSequence);
	
	// Set executor & create safety system
	auto &executor = Executor::instance();
	executor.setMainTask(safetySystem);
	
	// Start control system
	executor.run();
	
	sequencer.shutdown();
	usleep(3);
	if(sequencer.getState()!=state::terminated) {
		sequencer.abort();
	}
// 	while(sequencer.getState()!=state::terminated){
// 		usleep(100000);
// 		std::cout << ".";
// 	}
	
	std::cout << "Shuting down..." << std::endl;
		
	return 0;
}
