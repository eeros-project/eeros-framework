#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <vector>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

namespace eeros {

	namespace control {
		class TimeDomain;
	}

	class Executor : public Runnable {
		Executor();
	public:
		static constexpr int basePriority = 49;

		virtual ~Executor();
		static Executor& instance();
		void setPeriod(double period);
		void setMainTask(task::Periodic &mainTask);
		void add(task::Periodic task);
		void add(control::TimeDomain &timedomain);
		virtual void run();

		static void prefault_stack();
		static bool lock_memory();
		static bool set_priority(int nice);

		PeriodicCounter counter;

	private:
		void assignPriorities();

		logger::Logger<logger::LogWriter> log;
		double period;
		task::Periodic *mainTask;
		std::vector<task::Periodic> tasks;
	};

}

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
