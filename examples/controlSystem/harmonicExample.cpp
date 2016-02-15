#include <iostream>
#include <functional>
#include <unistd.h>

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/Version.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/core/System.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>


namespace {

	using Logger = eeros::logger::Logger<eeros::logger::LogWriter>;

	class periodic {
	public:
		periodic(double period, const char *name, std::function<void(Logger&)> task) :
			log('T'),
			lambda([this, task] () { task(log); }),
			periodicTask(name, period, lambda) { }
			eeros::task::Periodic & getTask() { return periodicTask; }

		Logger log;
		eeros::task::Lambda lambda;
		eeros::task::Periodic periodicTask;
	};

}

int main() {

	eeros::logger::StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	w.show();

	Logger log('M');

	log.trace() << "harmonic tasks example";
	log.trace() << "eeros " << eeros::Version::string;

	eeros::Executor &executor = eeros::Executor::instance();
	executor.setPeriod(1);

	auto ss = periodic(1, "SS", [](Logger &log) {
		log.trace() << (long)eeros::System::getTimeNs() << "\tSS";
	});
	executor.setMainTask(ss.getTask());

	auto t1 = periodic(1, "t1", [](Logger &log) {
		static int counter = 0;
		log.trace() << (long)eeros::System::getTimeNs() << "\t   t1";

		if (++counter >= 10) {
			counter = 0;
			//sleep(4);
		}
	});
	t1.periodicTask.addDefaultMonitor();

	auto t2 = periodic(2, "t2", [](Logger &log) {
		log.trace() << (long)eeros::System::getTimeNs() << "\t      t2";
	});
	t2.periodicTask.addDefaultMonitor();

	auto t4 = periodic(4, "t4", [](Logger &log) {
		log.trace() << (long)eeros::System::getTimeNs() << "\t         t4";
	});
	t2.getTask().after.push_back(t4.getTask());
	t1.getTask().after.push_back(t2.getTask());
	t4.periodicTask.addDefaultMonitor();

	auto t3 = periodic(5, "t3", [](Logger &log) {
		log.trace() << (long)eeros::System::getTimeNs() << "\t            t3";
	});
	t1.getTask().after.push_back(t3.getTask());
	t3.periodicTask.addDefaultMonitor();

	executor.add(t1.getTask());

	executor.run();

	return 0;
}
