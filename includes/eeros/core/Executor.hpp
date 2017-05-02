#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <vector>
#include <condition_variable>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/logger/Logger.hpp>

#ifdef ECMASTERLIB_FOUND
#include <EtherCATMain.hpp>
#endif


namespace eeros {

	namespace control {
		class TimeDomain;
	}

	namespace safety {
		class SafetySystem;
	};

	class Executor : public Runnable {
		Executor();
	public:
		static constexpr int basePriority = 49;

		virtual ~Executor();
		static Executor& instance();
#ifdef ECMASTERLIB_FOUND
		void syncWithEtherCATSTack(ethercat::EtherCATMain* etherCATStack);
#endif
		void setMainTask(task::Periodic &mainTask);
		void setMainTask(safety::SafetySystem &mainTask);
		void add(task::Periodic &task);
		void add(control::TimeDomain &timedomain);
		virtual void run();

		static void prefault_stack();
		static bool lock_memory();
		static bool set_priority(int nice);
		static void stop();

		PeriodicCounter counter;

	private:
		void assignPriorities();
;
#ifdef ECMASTERLIB_FOUND
		ethercat::EtherCATMain* etherCATStack;
		std::mutex* m;
		std::condition_variable* cv;
#endif
		
		logger::Logger log;
		double period;
		task::Periodic *mainTask;
		std::vector<task::Periodic> tasks;
	};

}

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
