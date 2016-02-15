#ifndef ORG_EEROS_TASK_PERIODIC_HPP_
#define ORG_EEROS_TASK_PERIODIC_HPP_

#include <vector>
#include <functional>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>

namespace eeros {
	namespace task {

		class Periodic {
		public:
			Periodic(const char *name, double period, Runnable &task, bool realtime = true, int nice = -1);
			Periodic(const char *name, double period, Runnable *task, bool realtime = true, int nice = -1);

			void addDefaultMonitor(double tolerance = 0.05);

			const char * getName();
			double getPeriod();
			Runnable& getTask();
			bool getRealtime();
			int getNice();
			void setNice(int value);

			std::vector<Periodic> before;
			std::vector<Periodic> after;

			std::vector<PeriodicCounter::MonitorFunc> monitors;

		private:
			const char *name;
			double period;
			Runnable *task;
			bool realtime;
			int nice;
		};

	}
}

#endif // ORG_EEROS_TASK_PERIODIC_HPP_
