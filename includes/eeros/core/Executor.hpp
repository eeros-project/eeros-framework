#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <vector>
#include <condition_variable>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/logger/Logger.hpp>

#ifdef USE_ETHERCAT
#include <EtherCATStack.hpp>
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
 * The executor is responsible for running periodics, e.g. time domains.
 * You have to set one periodic as the main task. From its period all the other periodics
 * which are added to the executor are executed as harmonic tasks. 
 * If the main task is not set, the executor will create a default main task with 
 * a chosen period.
 *
 * @since v0.6
 */
class Executor : public Runnable {
 public:
  virtual ~Executor();
  static Executor& instance();
  
  /**
   * Set the main task.
   * 
   * @param mainTask - periodic which will become the main task
   */
  void setMainTask(task::Periodic &mainTask);

  /**
   * Set the main task.
   * 
   * @param mainTask - periodic which will become the main task
   */
  void setMainTask(safety::SafetySystem &mainTask);
  
  /**
   * If no main task is set, the executor will create a default main task with 
   * a chosen period.
   * 
   * @param period - period of the default main task
   */
  void setExecutorPeriod(double period);
  task::Periodic* getMainTask();
  
  /**
   * Adds a periodic to the task list of the executor. The executor will 
   * periodically execute the runnable of the periodic.
   * 
   * @param task - periodic
   */
  void add(task::Periodic &task);
  
  /**
   * An instance of the class \ref Periodic will be created which in turn is 
   * added to the executor. The executor will periodically execute the runnable
   * of the periodic, which is the timedomain.
   * 
   * @param timedomain - timedomain
   */
  void add(control::TimeDomain &timedomain);
  
  virtual void run();

  static void prefault_stack();
  static bool lock_memory();
  static bool set_priority(int nice);
  static void stop();
  static constexpr int basePriority = 49;
  PeriodicCounter counter;
#ifdef USE_ETHERCAT
  void syncWithEtherCATSTack(ecmasterlib::EtherCATStack* etherCATStack);
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
  ecmasterlib::EtherCATStack* etherCATStack;
#endif
};

}

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
