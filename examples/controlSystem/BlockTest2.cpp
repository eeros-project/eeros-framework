#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/task/Lambda.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::hal;
using namespace eeros::task;

double period = 0.01;

class ControlSystem;
class SafetyPropertiesTest : public SafetyProperties {
public:
	SafetyPropertiesTest(ControlSystem& cs);
	
	ControlSystem& controlSystem;

	SafetyEvent seStartRunning;
	SafetyEvent seDoEmergency;
	
	SafetyLevel slEmergency;
	SafetyLevel slInitializing;
	SafetyLevel slRunning;
};

class ControlSystem {
public:
	ControlSystem(TimeDomain& td) : g(10), p("out"), td(td) {
		c.setValue(0.568);
		c.setName("constant");
		g.setName("gain");
		c.getOut().getSignal().setName("constant output");
		g.getOut().getSignal().setName("gain output");
		g.getIn().connect(c.getOut());
		p.getIn().connect(g.getOut());
		td.addBlock(c);
		td.addBlock(g);
		td.addBlock(p);
	}
	
	Constant<> c;
	Gain<> g;
	PeripheralOutput<> p;
	TimeDomain& td;
};

SafetyPropertiesTest::SafetyPropertiesTest(ControlSystem& cs) : 
	controlSystem(cs),
	seStartRunning("start running"),
	seDoEmergency("go to emergency"),
	slEmergency("emergency"),
	slInitializing("initializing"),
	slRunning("running")
	{	
	
	// ############ Add levels ############
	addLevel(slEmergency);
	addLevel(slInitializing);
	addLevel(slRunning);
	
	// ############ Add events to the levels ############
	slInitializing.addEvent(seStartRunning, slRunning, kPrivateEvent);
	addEventToLevelAndAbove(slInitializing, seDoEmergency, slEmergency, kPublicEvent);

	// Define and add level functions
	slEmergency.setLevelAction([&](SafetyContext* privateContext) {
		if(slEmergency.getNofActivations() == 1) controlSystem.td.stop();
	});
	
	slInitializing.setLevelAction([&](SafetyContext* privateContext) {
		controlSystem.td.stop();
		if(slInitializing.getNofActivations() * period > 3) {
			privateContext->triggerEvent(seStartRunning);
			controlSystem.td.start();
		}
	});
	
	// Define entry level
	setEntryLevel(slInitializing);	
}

int main(int argc, char **argv) {
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	
	log.info() << "Block Test 2 started...";
	
	// Get HAL instance and initialize
	HAL& hal = HAL::instance();
	hal.readConfigFromFile(&argc, argv);

	TimeDomain td("td1", period, true);
	ControlSystem controlSystem(td);
	
	SafetyPropertiesTest ssProperties(controlSystem);
	SafetySystem safetySys(ssProperties, period);
	
	td.registerSafetyEvent(safetySys, ssProperties.seDoEmergency);

	Lambda l1 ([&] () { });
	Periodic periodic("per1", 1, l1);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
		log.info() << controlSystem.g.getOut().getSignal();
	});
	
	// Create and run executor
	auto& executor = eeros::Executor::instance();
	executor.setMainTask(safetySys);
	executor.add(td);
	executor.add(periodic);
	executor.run();

	log.info() << "Test finished...";
}
