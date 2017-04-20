#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/XBoxInput.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;

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

int main() {
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	
	log.info() << "XBox Test started...";
	
	ControlSystem controlSystem;
	
	Periodic periodic("per1", period, controlSystem.td);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
		static int ticks = 0;
		if ((++ticks * period) < 0.5) return;
		ticks = 0;
		log.info() << controlSystem.xbox.getOut().getSignal() << controlSystem.xbox.getButtonOut().getSignal();
	});
		
	// Create and run executor
	auto& executor = eeros::Executor::instance();
	executor.setMainTask(periodic);
	executor.run();

	log.info() << "Test finished...";
}
