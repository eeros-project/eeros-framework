#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Transition.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::math;
using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;

double period = 0.05;

class ControlSystem {
public:
	ControlSystem() : tdFast("td fast", period, true), tdSlow("td slow", 100 * period, true), c1(1), t1(100), t2(0.01) {
		i1.getIn().connect(c1.getOut());
		i1.setInitCondition(0);
		i1.enable();
		i1.getOut().getSignal().setName("integrator output");
		t1.inBlock.getIn().connect(i1.getOut());
		t1.outBlock.getOut().getSignal().setName("transition block t1 output");
		t2.inBlock.getIn().connect(t1.outBlock.getOut());
		t2.outBlock.getOut().getSignal().setName("transition block t2 output");
		t2.outBlock.getIn().connect(c1.getOut());
		tdSlow.addBlock(c1);
		tdSlow.addBlock(i1);
		tdSlow.addBlock(t1.inBlock);
		tdFast.addBlock(t1.outBlock);
		tdFast.addBlock(t2.inBlock);
		tdSlow.addBlock(t2.outBlock);
		Executor::instance().add(tdFast);
		Executor::instance().add(tdSlow);
	}
	TimeDomain tdFast, tdSlow;
	Constant<> c1;
	I<> i1;
	Transition<> t1;
	Transition<> t2;
};

class SafetyPropertiesTest : public SafetyProperties {
public:
	SafetyPropertiesTest() : slState1("state 1") {
		addLevel(slState1);
		setEntryLevel(slState1);	
	};
	SafetyLevel slState1;
};

int main() {
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	
	log.info() << "Transitions block test 1 started...";
	
	ControlSystem controlSystem;

	// Create and initialize safety system
	SafetyPropertiesTest ssProperties;
	SafetySystem safetySys(ssProperties, period);
		
	// create periodic function for logging
	Lambda l1 ([&] () { });
	Periodic periodic("per1", 0.5, l1);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
//		log.warn() << controlSystem.i1.getOut().getSignal();
		log.info() << controlSystem.t1.outBlock.getOut().getSignal() << "   " << controlSystem.t2.outBlock.getOut().getSignal();
//  		log.info() << controlSystem.i1.getOut().getSignal() << "   " << controlSystem.t1.outBlock.getOut().getSignal() << "   " << controlSystem.t2.outBlock.getOut().getSignal();
	});

	// Create and run executor
	auto& executor = eeros::Executor::instance();
	executor.setMainTask(safetySys);
	executor.add(periodic);
	executor.run();

	log.info() << "Test finished...";
}
