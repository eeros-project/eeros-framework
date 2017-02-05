#include <iostream>
#include <thread>

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/Version.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/core/System.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>

/*
 * example with one harmonic task
 */ 
namespace {
	using Logger = eeros::logger::Logger;
}

int main() {
	const double dt = 0.005;
	eeros::logger::StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	w.show();

	Logger log('M');

	log.trace() << "harmonic tasks example";
	log.trace() << "eeros " << eeros::Version::string;

	eeros::Executor& executor = eeros::Executor::instance();

	eeros::task::Lambda ls ([&] () { });
	eeros::task::Periodic ss("ss", dt, ls);
	executor.setMainTask(ss);
	ss.addDefaultMonitor();
	ss.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks < 1000) return;
		ticks = 0;
		log.info() << "ss: period max: " << c.period.max << "   period min: " << c.period.min << "   period mean: " << c.period.mean;
		c.reset();
	});

	eeros::task::Lambda l1 ([&] () { });
	eeros::task::Periodic t1("t1", dt, l1);
	
	t1.addDefaultMonitor();
	t1.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks < 1000) return;
		ticks = 0;
		log.info() << "ss: period max: " << c.period.max << "   period min: " << c.period.min << "   period mean: " << c.period.mean;
		c.reset();
	});
	
 	executor.add(t1);
 
	executor.run();

	return 0;
}
