#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/control/drives/ODriveInput_USB.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::task;

double period = 0.01;

void signalHandler(int signum){
	SafetySystem::exitHandler();
}

class ControlSystem {
 public:
  ControlSystem() : 
	odrive_input(0x208c397d4d4d, 4096, 20, true), 
	td("td", period, true) 
  {
	td.addBlock(odrive_input);
  }
    
  ODriveInput_USB odrive_input;
  TimeDomain td;
};

class PPSafetyProperties : public SafetyProperties {
 public:
  PPSafetyProperties() : slState1("state 1") {
    addLevel(slState1);
    setEntryLevel(slState1);	
  };
  SafetyLevel slState1;
};

int main() {
	Logger::setDefaultStreamLogger(std::cout);
	Logger log = Logger::getLogger();
	log.show(LogLevel::TRACE);
	log.info() << "Program started - Read out Baumer sensors data";
	
	ControlSystem cs;
	Periodic p1("p1", period, cs.td);
	PPSafetyProperties sp;
	SafetySystem ss(sp, period);
	
	// create periodic function for logging
	Lambda l1 ([&] () { });
	Periodic p2("p2", period, l1);
	p2.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
		log.info() << cs.odrive_input.getVelActOut().getSignal().getValue() ;
	});
	
	auto& executor = Executor::instance();
	executor.setMainTask(ss);
	p1.after.push_back(p2); // make sure that logging happens after running of path planner
	executor.add(p1);
	executor.run();
			
	log.info() << "Program finished...";
	return 0;
}

