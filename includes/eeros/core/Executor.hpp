#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <vector>
#include <condition_variable>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/logger/Logger.hpp>

#ifdef USE_ETHERCAT
#include <EcMasterlibMain.hpp>
#endif

#ifdef USE_ROS
#include <ros/ros.h>
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
#ifdef USE_ETHERCAT
		void syncWithEtherCATSTack(ecmasterlib::EcMasterlibMain* etherCATStack);
#endif
		void setMainTask(task::Periodic &mainTask);
		void setMainTask(safety::SafetySystem &mainTask);
		task::Periodic* getMainTask();
		void add(task::Periodic &task);
		void add(control::TimeDomain &timedomain);
		virtual void run();

		static void prefault_stack();
		static bool lock_memory();
		static bool set_priority(int nice);
		static void stop();

		
		PeriodicCounter counter;
		
		
#ifdef USE_ROS
		void syncWithRosTime();
		void syncWithRosTopic(ros::CallbackQueue* syncRosCallbackQueue);
		ros::CallbackQueue* syncRosCallbackQueue;
#endif
		

	private:
		void assignPriorities();
;
#ifdef USE_ETHERCAT
		ecmasterlib::EcMasterlibMain* etherCATStack;
		std::mutex* m;
		std::condition_variable* cv;
#endif
		bool syncWithEtherCatStackIsSet;
		bool syncWithRosTimeIsSet;
		bool syncWithRosTopicIsSet;
		logger::Logger log;
		double period;
		task::Periodic* mainTask;
		std::vector<task::Periodic> tasks;
	};

}

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
