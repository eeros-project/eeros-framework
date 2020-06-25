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

/**
 * The executor is responsible for running the time domains.
 *
 * @since v0.6
 */
class Executor : public Runnable {
 public:
  virtual ~Executor();
  static Executor& instance();
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
  static constexpr int basePriority = 49;
  PeriodicCounter counter;
#ifdef USE_ETHERCAT
  void syncWithEtherCATSTack(ecmasterlib::EcMasterlibMain* etherCATStack);
#endif
#ifdef USE_ROS
  void syncWithRosTime();
  void syncWithRosTopic(ros::CallbackQueue* syncRosCallbackQueue);
  ros::CallbackQueue* syncRosCallbackQueue;
#endif

 private:
  Executor();
  void assignPriorities();
  double period;
  task::Periodic* mainTask;
  std::vector<task::Periodic> tasks;
  bool syncWithEtherCatStackIsSet;
  bool syncWithRosTimeIsSet;
  bool syncWithRosTopicIsSet;
  logger::Logger log;
#ifdef USE_ETHERCAT
  ecmasterlib::EcMasterlibMain* etherCATStack;
  std::mutex* m;
  std::condition_variable* cv;
#endif
};

}

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
