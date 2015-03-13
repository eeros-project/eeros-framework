#include <eeros/core/PeriodicCounter.hpp>
using namespace eeros;

PeriodicCounter::PeriodicCounter(double period, unsigned logger_category) :
	counter_period(period), max_period(1.2*period), min_period(0.8*period),
	max_jitter(0.2*period), min_jitter(-max_jitter),
	max_run(0.8*period), min_run(0),
	warn_counter(0), rate_limiter(0),
	log(logger_category)
{
	reset();
	start = clk::now();
}

void PeriodicCounter::tick() {
	last = start;
	start = clk::now();
}

void PeriodicCounter::tock() {
	time_point stop = clk::now();
	double new_period = std::chrono::duration<double>(start - last).count();
	double new_run = std::chrono::duration<double>(stop - start).count();
	double new_jitter = (new_period - counter_period);
	
	period.add(new_period);
	run.add(new_run);
	jitter.add(new_jitter);
	
	bool warn = false;
	bool first = (rate_limiter == 0);
	if (new_period < min_period || new_period > max_period) {
		if (first) log.warn() << "period: " << new_period;
		warn = true;
	}
	if (new_jitter < min_jitter || new_jitter > max_jitter) {
		if (first) log.warn() << "jitter: " << new_jitter;
		warn = true;
	}
	if (new_run < min_run || new_run > max_run) {
		if (first) log.warn() << "run:    " << new_run;
		warn = true;
	}
	if (warn) {
		warn_counter++;
		
		if (rate_limiter == 0)
			rate_limiter = (int)(1 / counter_period);
	}
	if (rate_limiter == 1)
	{
		log.warn() << warn_counter << " warnings not shown";
		warn_counter = 0;
		rate_limiter = 0;
	}
	else if (rate_limiter > 0) {
		rate_limiter--;
	}
}

void PeriodicCounter::reset() {
	period.reset();
	jitter.reset();
	run.reset();
}
