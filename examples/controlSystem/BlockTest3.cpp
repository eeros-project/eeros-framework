#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/task/Lambda.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;

double period = 0.1;

class ControlSystem {
public:
	ControlSystem() : c1(0.25), c2(2.5), sw(0) {
		i.getIn().connect(c1.getOut());
		i.enable();
		sw.getIn(0).connect(i.getOut());
		sw.getIn(1).connect(c2.getOut());
		sw.getOut().getSignal().setName("switch output");
	}

	Constant<> c1, c2;
	Switch<> sw;
	I<> i;
};

class SafetyPropertiesTest : public SafetyProperties {
public:
	SafetyPropertiesTest(ControlSystem& cs) : 
		slState1("state 1"), slState2("state 2"), 
		seGoto1("switch to state 1"), seGoto2("switch to state 2"), 
		cs(cs) {
			
		// ############ Add levels ############
		addLevel(slState1);
		addLevel(slState2);
		
		// ############ Add events to the levels ############
		slState1.addEvent(seGoto2, slState2, kPublicEvent);
		slState2.addEvent(seGoto1, slState1, kPrivateEvent);
		
		// Define and add level functions
		slState1.setLevelAction([&](SafetyContext* privateContext) {
			if(slState1.getNofActivations() == 1) {
				cs.i.setInitCondition(0);
				cs.sw.switchToInput(0);
				cs.sw.setCondition(1.5, 0.1, 1);
				cs.sw.arm();
			}
		});
		
		slState2.setLevelAction([&](SafetyContext* privateContext) {
			if(slState2.getNofActivations() * period > 3) {
				cs.sw.switchToInput(0);
				privateContext->triggerEvent(seGoto1);
			}
		});
		
		// Define entry level
		setEntryLevel(slState1);	
	};
	SafetyLevel slState1, slState2;
	SafetyEvent seGoto1, seGoto2;
	ControlSystem& cs;
};

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
	
	log.info() << "Block test 3 started...";
	
	ControlSystem controlSystem;

	// Create and initialize safety system
	SafetyPropertiesTest ssProperties(controlSystem);
	SafetySystem safetySys(ssProperties, period);
	
	// create time domain and add blocks of control system
	TimeDomain td("td1", period, true);
	td.addBlock(controlSystem.c1);
	td.addBlock(controlSystem.c2);
	td.addBlock(controlSystem.sw);
	td.addBlock(controlSystem.i);
	controlSystem.sw.registerSafetyEvent(safetySys, ssProperties.seGoto2);
	
	// create periodic function for logging
	Lambda l1 ([&] () { });
	Periodic periodic("per1", 0.5, l1);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
		log.info() << controlSystem.sw.getOut().getSignal();
	});
	
	// Create and run executor
	auto& executor = eeros::Executor::instance();
	executor.setMainTask(safetySys);
	executor.add(td);
	executor.add(periodic);
	executor.run();

	log.info() << "Test finished...";
}
