#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/HAL.hpp>
#include "HalTest1.hpp"

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::task;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::sequencer;

int main(int argc, char **argv){
	// Create and initialize logger
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	w.show();
	
	log.info() << "HAL simulator test started...";
  
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);
	
// 	hal.callOutputFeature("pwm1", "setPwmFrequency", 100.0);
	
	// Create safety and control system
	MyControlSystem cs(dt);
	MySafetyProperties safetyProperties;
	SafetySystem safetySystem(safetyProperties, dt);
	
	// Sequencer
	Sequencer sequencer;
	MyMainSequence mainSequence(&sequencer, cs);
	sequencer.start(&mainSequence);
	
	// Set executor and run
	auto &executor = Executor::instance();
	executor.setMainTask(safetySystem);
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
	
	log.info() << "end...";
		
	return 0;
}