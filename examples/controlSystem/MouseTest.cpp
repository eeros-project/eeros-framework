#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/MouseInput.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;

double period = 0.01;

class ControlSystem {
public:
	ControlSystem() : mouse("/dev/input/event7"), td("td1", period, true) {
		mouse.setName("mouse");
		mouse.getOut().getSignal().setName("position");
		mouse.getButtonOut().getSignal().setName("events");
		td.addBlock(mouse);
	}

	MouseInput mouse;
	TimeDomain td;
};

int main() {
	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	
	log.info() << "Mouse Test started ...";
	
	ControlSystem controlSystem;
	
	Periodic periodic("per1", period, controlSystem.td);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
		static int ticks = 0;
		if ((++ticks * period) < 0.5) return;
		ticks = 0;
		log.info() << controlSystem.mouse.getOut().getSignal() << controlSystem.mouse.getButtonOut().getSignal();
	});
		
	// Create and run executor
	auto& executor = eeros::Executor::instance();
	executor.setMainTask(periodic);
	executor.run();

	log.info() << "Test finished...";
}
