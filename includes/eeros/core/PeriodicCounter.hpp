#ifndef ORG_EEROS_CORE_PERIODICCOUNTER_HPP_
#define ORG_EEROS_CORE_PERIODICCOUNTER_HPP_

#include <chrono>
#include <vector>
#include <functional>

#include <eeros/core/Statistics.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

namespace eeros {

	class PeriodicCounter {
		using clk = std::chrono::steady_clock;
		using time_point = clk::time_point;

	public:
		using Logger = logger::Logger;
		using MonitorFunc = std::function<void(PeriodicCounter&, Logger&)>;

		PeriodicCounter(double period = 0, unsigned logger_category = 0);
		
		void setPeriod(double period);
		void setResetTime(double sec);
		void addDefaultMonitor(double tolerance = 0.05);

		void tick();
		void tock();
		void reset();

		void operator >> (eeros::logger::LogEntry &event);
		void operator >> (eeros::logger::LogEntry &&event);

		Statistics period;
		Statistics jitter;
		Statistics run;

		std::vector<MonitorFunc> monitors;

		static void addDefaultMonitor(std::vector<MonitorFunc> &monitors, double period, double tolerance = 0.05);

	private:
		double counter_period;
		double reset_after;
		bool first;
		int reset_counter;
		time_point start;
		time_point last;
		Logger log;
	};
}

#endif // ORG_EEROS_CORE_PERIODICCOUNTER_HPP_
