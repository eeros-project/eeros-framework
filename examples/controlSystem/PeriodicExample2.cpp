#include <iostream>
#include <thread>

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/Version.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/core/System.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>

/*
 * example with several harmonic tasks
 */ 
namespace {
	using Logger = eeros::logger::Logger<eeros::logger::LogWriter>;
}

int main() {
	const double dt = 1.0;
	eeros::logger::StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	w.show();

	Logger log('M');
	Logger log2('T');

	log.trace() << "harmonic tasks example";
	log.trace() << "eeros " << eeros::Version::string;

	eeros::Executor &executor = eeros::Executor::instance();

	eeros::task::Lambda ls ([&] () { 		
		log.trace() << (long)eeros::System::getTimeNs() << "\tss";
		for (int i = 0; i < 1; i++) {
			log.trace() << "\tdo some work in ss";
			std::this_thread::sleep_for(std::chrono::microseconds((int)(dt / 100)));
		}
	});
	eeros::task::Periodic ss("ss", dt, ls);
	executor.setMainTask(ss);
	ss.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks < 10) return;
		ticks = 0;
		log.info() << "ss: period max: " << c.period.max*1000 << "   run max: " << c.run.max*1000 << "   run mean: " << c.run.mean*1000;
		c.reset();
	});

	eeros::task::Lambda l1 ([&] () { 		
		log2.trace() << (long)eeros::System::getTimeNs() << "\tt1";
		for (int i = 0; i < 4; i++) {
			log2.trace() << "\tdo some work in t1";
			std::this_thread::sleep_for(std::chrono::microseconds((int)(dt / 100)));
		}
	});
	eeros::task::Periodic t1("t1", dt, l1);
	
	t1.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks < 10) return;
		ticks = 0;
		log.info() << "t1: period max: " << c.period.max*1000 << "   run max: " << c.run.max*1000 << "   run mean: " << c.run.mean*1000;
		c.reset();
	});
	
	eeros::task::Lambda l2 ([&] () { 		
		log2.trace() << (long)eeros::System::getTimeNs() << "\tt2";
		for (int i = 0; i < 3; i++) {
			log2.trace() << "\tdo some work in t2";
			std::this_thread::sleep_for(std::chrono::microseconds((int)(dt / 100)));
		}
	});
	eeros::task::Periodic t2("t2", 2 * dt, l2);
	t2.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks < 10) return;
		ticks = 0;
		log.info() << "t2: period max: " << c.period.max*1000 << "   run max: " << c.run.max*1000 << "   run mean: " << c.run.mean*1000;
		c.reset();
	});

	eeros::task::Lambda l4 ([&] () { 		
		log2.trace() << (long)eeros::System::getTimeNs() << "\tt4";
		for (int i = 0; i < 2; i++) {
			log2.trace() << "\tdo some work in t4";
			std::this_thread::sleep_for(std::chrono::microseconds((int)(dt / 100)));
		}
	});
	eeros::task::Periodic t4("t2", 4 * dt, l4);
	t2.after.push_back(t4);
	t1.after.push_back(t2);
	t4.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks < 10) return;
		ticks = 0;
		log.info() << "t4: period max: " << c.period.max*1000 << "   run max: " << c.run.max*1000 << "   run mean: " << c.run.mean*1000;
		c.reset();
	});

	eeros::task::Lambda l3 ([&] () { 		
		log2.trace() << (long)eeros::System::getTimeNs() << "\tt3";
		for (int i = 0; i < 4; i++) {
			log2.trace() << "\tdo some work in t3";
			std::this_thread::sleep_for(std::chrono::microseconds((int)(dt / 100)));
		}
	});
	eeros::task::Periodic t3("t3", 5 * dt, l3);
	t1.after.push_back(t3);
	t3.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks < 10) return;
		ticks = 0;
		log.info() << "t3: period max: " << c.period.max*1000 << "   run max: " << c.run.max*1000 << "   run mean: " << c.run.mean*1000;
		c.reset();
	});

	eeros::task::Lambda l5 ([&] () { 		
		log2.trace() << (long)eeros::System::getTimeNs() << "\tt5";
		for (int i = 0; i < 6; i++) {
			log2.trace() << "\tdo some work in t5";
			std::this_thread::sleep_for(std::chrono::microseconds((int)(dt / 100)));
		}
	});
	eeros::task::Periodic t5("t5", 3 * dt, l5);
	t5.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
		static int ticks = 0;
		if (++ticks < 10) return;
		ticks = 0;
		log.info() << "t5: period max: " << c.period.max*1000 << "   run max: " << c.run.max*1000 << "   run mean: " << c.run.mean*1000;
		c.reset();
	});

 	executor.add(t1);
 	executor.add(t5);

	executor.run();

	return 0;
}
