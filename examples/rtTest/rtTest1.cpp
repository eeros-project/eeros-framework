#include <iostream>
#include <functional>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <ctime>
#include <iomanip>

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/Version.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetySystem.hpp>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
namespace {
	using Logger = eeros::logger::Logger;
}

class SafetyPropertiesTest : public SafetyProperties {
public:
	SafetyPropertiesTest() : slOff("off") {	
		addLevel(slOff);
		setEntryLevel(slOff);
	}
	
	SafetyLevel slOff;
};

int seconds = 10;

int main(int argc, char *argv[]) {
	int c;
	while((c = getopt(argc, argv, "s:")) != -1) {
		switch (c) {
			case 's':
				seconds = atoi(optarg);
				break;
			case '?':
				if (optopt == 's')
					std::cerr << "Option " << char(optopt) << " requires an argument.\n" << std::endl;
				else if (isprint (optopt))
					std::cerr << "Unknown option " << char(optopt) << std::endl;
				else
					std::cerr << "Unknown option character " << char(optopt) << std::endl;
				return -1;
			default:
				abort ();
		}
	}
	
	const double dt = 0.001;
	eeros::logger::StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	w.show();

	Logger log('M');

	log.trace() << "measure periodic execution";
	log.trace() << "eeros " << eeros::Version::string;

	// Create and initialize safety system
	SafetyPropertiesTest ssProperties;
	SafetySystem safetySys(ssProperties, dt);

	auto executor = eeros::Executor::instance();
	eeros::task::Periodic ss("ss", dt, safetySys);
	executor.setMainTask(ss);
	ss.monitors.push_back([&](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks % 1000 == 0) log.info() << "ss: period max: " << c.period.max << "   period min: " << c.period.min << "   period mean: " << c.period.mean;
		if (ticks > seconds / dt) executor.stop();
	});

	executor.run();

	return 0;
}
