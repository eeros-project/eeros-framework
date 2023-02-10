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

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
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

  /**
   * Get the main executor instance as a singleton
   */
  static Executor& instance();


  /**
   * Copy and assignemt for a singleton is disabled
   */
  Executor(Executor const&) = delete;
  void operator=(Executor const&) = delete;

  
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

  /**
   * Get the main task.
   * 
   * @return main task
   */
  task::Periodic* getMainTask();
  
  /**
   * Adds a periodic to the task list of the executor. The executor will 
   * periodically execute the runnable of the periodic.
   * 
   * @param task - periodic
   */
  void add(task::Periodic &task);
  
  /**
   * An instance of the class \ref task::Periodic will be created which in turn is 
   * added to the executor. The executor will periodically execute the runnable
   * of the periodic, which is the timedomain.
   * 
   * @param timedomain - timedomain
   */
  void add(control::TimeDomain &timedomain);
  
  /*
   * Allocates memory chunck on the stack. Used for later locking.
   */
  static void prefault_stack();

  /*
   * Locks all pages associated with the current process, which
   * prevents the pages from being swapped out later.
   */
  static bool lock_memory();

  /**
   * Sets the priority of the executor task.
   *
   * @param nice - priority
   */
  static bool set_priority(int nice);

  /**
   * Starts the executor.
   */
  virtual void run();

  /**
   * Stops the executor.
   */
  static void stop();

  static constexpr int basePriority = 49;
  PeriodicCounter counter;

#ifdef USE_ETHERCAT
  /**
   * The executor will run in synch with the EtherCAT stack.
   *
   * @param etherCATStack - reference to the EtherCAT stack
   */
  void syncWithEtherCATSTack(ecmasterlib::EcMasterlibMain* etherCATStack);
#endif
#if defined USE_ROS || defined USE_ROS2
  /**
   * Caused the executor to fetch its time base from ROS time.
   * Further, all blocks using the system time base will also switch.
   * Do not use this together with EtherCAT.
   */
  void syncWithRosTime();
#endif
#ifdef USE_ROS
  void syncWithRosTopic(ros::CallbackQueue* syncRosCallbackQueue);
  ros::CallbackQueue* syncRosCallbackQueue;
#endif
#ifdef USE_ROS2
  /**
   * Called by a ROS subscriber, registers a subscriber.
   * Do not use this together with EtherCAT.
   *
   * @param node - ROS node as a shared ptr
   * @param sync - when true, executor is triggered by ROS subscriber
   */
  rclcpp::CallbackGroup::SharedPtr registerSubscriber(rclcpp::Node::SharedPtr node, const bool sync = false);
  /**
   * Called by a ROS subscriber, will trigger the executor upon receiving a ROS message.
   */
  void handleTopic();
#endif

 private:
  Executor();
  void assignPriorities();
  double period;
  task::Periodic* mainTask;
  std::vector<task::Periodic> tasks;
  bool syncWithEtherCatStackSet;
  bool syncWithRosTimeSet;
  bool syncWithRosTopicSet;
  bool running = true;
  logger::Logger log;
#ifdef USE_ROS2
  rclcpp::Executor::SharedPtr subscriberExecutor;
  std::shared_ptr<std::thread> subscriberThread;
  std::condition_variable cv;
  std::mutex cv_m;
  int count;
#endif
#ifdef USE_ETHERCAT
  ecmasterlib::EcMasterlibMain* etherCATStack;
#endif
};

}

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
