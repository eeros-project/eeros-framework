#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PathPlannerTrapezoid.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/task/Lambda.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::math;

double period = 1;

class ControlSystem {
public:
	ControlSystem() : pp(1.0, 0.2, 0.2, period), td("td1", period, true) {
		pp.getPosOut().getSignal().setName("pp pos out");
		pp.setInitPos({0, 0});
		Matrix<2,1,double> dest{10, 20};
		pp.move(dest);
		td.addBlock(pp);
		eeros::Executor::instance().add(td);
	}

	PathPlannerTrapezoid<Matrix<2,1,double>> pp;
	TimeDomain td;
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
	
	log.info() << "Pathplanner test 1 started...";
	
	ControlSystem controlSystem;

	// Create and initialize safety system
	SafetyPropertiesTest ssProperties;
	SafetySystem safetySys(ssProperties, period);
		
	// create periodic function for logging
	Lambda l1 ([&] () { });
	Periodic periodic("per1", period, l1);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
		log.info() << controlSystem.pp.getPosOut().getSignal(); 
	});
	
	// Create and run executor
	auto& executor = eeros::Executor::instance();
	executor.setMainTask(safetySys);
	executor.add(periodic);
	executor.run();

	log.info() << "Test finished...";
}
