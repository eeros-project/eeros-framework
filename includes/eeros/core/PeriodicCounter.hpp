#ifndef ORG_EEROS_CORE_PERIODICCOUNTER_HPP_
#define ORG_EEROS_CORE_PERIODICCOUNTER_HPP_

#include <eeros/core/Statistics.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <chrono>

namespace eeros {

	class PeriodicCounter {
		using clk = std::chrono::high_resolution_clock;
		using time_point = clk::time_point;

	public:
		PeriodicCounter(double period, unsigned logger_category = 0);
		
		void tick();
		void tock();
		void reset();

		Statistics period;
		Statistics jitter;
		Statistics run;

	private:
		double counter_period, max_period, min_period;
		double max_jitter, min_jitter;
		double max_run, min_run;
		int warn_counter, rate_limiter;
		time_point start;
		time_point last;
		logger::Logger<logger::LogWriter> log;
	};
};

#endif // ORG_EEROS_CORE_PERIODICCOUNTER_HPP_
