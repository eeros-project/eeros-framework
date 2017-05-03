#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/XBoxInput.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::safety;

double period = 0.01;

class ControlSystem {
public:
	ControlSystem() : xbox("/dev/input/js0"), td("td1", period, true) {
		xbox.setName("xbox");
		xbox.getOut().getSignal().setName("xbox signal");
		td.addBlock(xbox);
	}

	XBoxInput xbox;
	TimeDomain td;
};

class MySafetyProperties : public SafetyProperties {
public:
	MySafetyProperties() : slFirst("first level"), slSecond("second level"), seGoUp("go to second level"), seGoDown("go to first level") {
		// ############ Define critical outputs ############
		Input<bool>* in1 = HAL::instance().getLogicInput("XBoxButtonX", false);
		criticalInputs = { in1 };

		// ############ Add levels ############
		addLevel(slFirst);
		addLevel(slSecond);
		
		// ############ Define input states and events for all levels ############
		slFirst.setInputActions({ check(in1, false, seGoUp) });
		slSecond.setInputActions({ check(in1, true, seGoDown) });
		
		// ############ Add events to the levels ############
		slFirst.addEvent(seGoUp, slSecond, kPrivateEvent);
		slSecond.addEvent(seGoDown, slFirst, kPrivateEvent);

		setEntryLevel(slFirst);
	}
	virtual ~MySafetyProperties() { }
	SafetyLevel slFirst;
	SafetyLevel slSecond;
	SafetyEvent seGoUp;
	SafetyEvent seGoDown;
};

int main() {
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	
	log.info() << "XBox Test started...";
	
	ControlSystem controlSystem;
	MySafetyProperties safetyProperties;
	SafetySystem safetySystem(safetyProperties, period);
	
	Periodic periodic("per1", period, controlSystem.td);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
		static int ticks = 0;
		if ((++ticks * period) < 0.5) return;
		ticks = 0;
		log.info() << controlSystem.xbox.getOut().getSignal() << controlSystem.xbox.getButtonOut().getSignal();
	});
		
	// Create and run executor
	auto& executor = eeros::Executor::instance();
	executor.setMainTask(safetySystem);
	executor.add(periodic);
	executor.run();

	log.info() << "Test finished...";
}
