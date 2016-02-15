#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/logger/Pretty.hpp>
using namespace eeros;

PeriodicCounter::PeriodicCounter(double period, unsigned logger_category) :
	log(logger_category), reset_after(20)
{
	setPeriod(period);
	start = clk::now();
	first = true;
}

void PeriodicCounter::setPeriod(double period) {
	counter_period = period;
	reset();
}

void PeriodicCounter::setResetTime(double sec) {
	reset_after = sec;
}

void PeriodicCounter::addDefaultMonitor(double tolerance) {
	PeriodicCounter::addDefaultMonitor(monitors, counter_period, tolerance);
}

void PeriodicCounter::tick() {
	last = start;
	start = clk::now();
}

void PeriodicCounter::tock() {
	if (first) {
		first = false;
		return;
	}

	time_point stop = clk::now();
	double new_period = std::chrono::duration<double>(start - last).count();
	double new_run = std::chrono::duration<double>(stop - start).count();
	double new_jitter = (new_period - counter_period);
	
	period.add(new_period);
	run.add(new_run);
	jitter.add(new_jitter);
	
	for (auto &func: monitors)
		func(*this, log);

	if (reset_counter <= 0) {
		*this >> log.trace();
		reset();
	}
	else {
		reset_counter--;
	}
}

void PeriodicCounter::reset() {
	period.reset();
	jitter.reset();
	run.reset();
	reset_counter = (int)(reset_after / counter_period);
}

void PeriodicCounter:: operator >> (eeros::logger::LogEntry<logger::LogWriter> &event) {
	using namespace eeros::logger;

	auto l = [](LogEntry<LogWriter> &e, Statistics &x) -> decltype(e) {
		return e << pretty(x.mean) << "\t" << pretty(x.variance) << "\t" << pretty(x.min) << "\t" << pretty(x.max);
	};

	event << "stats:\t     mean\t variance\t      min\t      max" << endl;

	event << "period\t";
	l(event, period) << endl;

	event << "jitter\t";
	l(event, jitter) << endl;

	event << "run   \t";
	l(event, run) << endl;

	event << "count = " << period.count;
}

void PeriodicCounter:: operator >> (eeros::logger::LogEntry<logger::LogWriter> &&event) {
	*this >> event;
}


void PeriodicCounter::addDefaultMonitor(std::vector<MonitorFunc> &monitors, double period, double tolerance){
	double Tmin = period * (1 - tolerance);
	double Tmax = period * (1 + tolerance);

	monitors.push_back([Tmin, Tmax](PeriodicCounter& counter, Logger& log){
		double T = counter.period.last;
		if (T < Tmin || T > Tmax) {
			auto e = log.warn();
			e << "last period was " << T << eeros::logger::endl;
			counter >> e;
		}
	});

}
