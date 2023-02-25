#include <algorithm>
#include <stdexcept>
#include <chrono>
#include <vector>
#include <memory>
#include <cmath>
#include <thread>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Async.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/task/HarmonicTaskList.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/SafetySystem.hpp>
#ifdef USE_ROS
#include <ros/callback_queue_interface.h>
#include <ros/callback_queue.h>
#endif

using namespace eeros;
using namespace std::chrono;

namespace {

using Logger = logger::Logger;

struct TaskThread {
  TaskThread(double period, task::Periodic &task, task::HarmonicTaskList tasks) 
      : taskList(tasks), async(taskList, task.getRealtime(), task.getNice()) {
    async.counter.setPeriod(period);
    async.counter.monitors = task.monitors;
  }
  task::HarmonicTaskList taskList;
  task::Async async;
};

template < typename F >
void traverse(std::vector<task::Periodic> &tasks, F func) {
  for (auto &t: tasks) {
    func(&t);
    traverse(t.before, func);
    traverse(t.after, func);
  }
}

void createThread(Logger &log, task::Periodic &task, task::Periodic &baseTask, std::vector<std::shared_ptr<TaskThread>> &threads, std::vector<task::Harmonic> &output);

void createThreads(Logger &log, std::vector<task::Periodic> &tasks, task::Periodic &baseTask, std::vector<std::shared_ptr<TaskThread>> &threads, task::HarmonicTaskList &output) {
  for (task::Periodic &t: tasks) {
    createThread(log, t, baseTask, threads, output.tasks);
  }
}

void createThread(Logger &log, task::Periodic &task, task::Periodic &baseTask, std::vector<std::shared_ptr<TaskThread>> &threads, std::vector<task::Harmonic> &output) {
  int k = static_cast<int>(task.getPeriod() / baseTask.getPeriod());
  double actualPeriod = k * baseTask.getPeriod();
  double deviation = std::abs(task.getPeriod() - actualPeriod) / task.getPeriod();
  task::HarmonicTaskList taskList;

  if (task.before.size() > 0) {
    createThreads(log, task.before, task, threads, taskList);
  }
  taskList.add(task.getTask());
  if (task.after.size() > 0) {
    createThreads(log, task.after, task, threads, taskList);
  }

  if (task.getRealtime())
    log.trace() << "creating harmonic realtime task '" << task.getName()
          << "' with period " << actualPeriod << " sec (k = "
          << k << ") and priority " << ((int)(Executor::basePriority) - task.getNice())
          << " based on '" << baseTask.getName() << "'";
  else
    log.trace() << "creating harmonic task '" << task.getName() << "' with period "
          << actualPeriod << " sec (k = " << k << ")"
          << " based on '" << baseTask.getName() << "'";

  if (deviation > 0.01) throw std::runtime_error("period deviation too high");

  if (task.getRealtime() && task.getNice() <= 0)
    throw std::runtime_error("priority not set");

  if (taskList.tasks.size() == 0)
    throw std::runtime_error("no task to execute");

  threads.push_back(std::make_shared<TaskThread>(actualPeriod, task, taskList));
  output.emplace_back(threads.back()->async, k);
}
}

Executor::Executor() 
    : period(0), mainTask(nullptr), syncWithEtherCatStackSet(false),
      syncWithRosTimeSet(false), syncWithRosTopicSet(false),
      log(logger::Logger::getLogger('E')) { }

Executor::~Executor() { }

Executor& Executor::instance() {
  static Executor executor;
  return executor;
}


#ifdef USE_ETHERCAT
void Executor::syncWithEtherCATSTack(ecmasterlib::EcMasterlibMain* etherCATStack) {
  syncWithEtherCatStackSet = true;
  this->etherCATStack = etherCATStack;
}
#endif


void Executor::setMainTask(task::Periodic &mainTask) {
  if (this->mainTask != nullptr)
    throw std::runtime_error("you can only define one main task per executor");
  period = mainTask.getPeriod();
  counter.setPeriod(period);
  this->mainTask = &mainTask;
}

void Executor::setMainTask(safety::SafetySystem &ss) {
  task::Periodic *task = new task::Periodic("safety system", ss.getPeriod(), ss, true);
  setMainTask(*task);
}

void Executor::setExecutorPeriod(double period) {
  if (this->mainTask != nullptr)
    throw std::runtime_error("set the executor period only in case you dont't have a main task");
  task::Periodic *task = new task::Periodic("default main task", period, new task::Lambda(), true);
  setMainTask(*task);
}

task::Periodic* Executor::getMainTask() {
  return mainTask;
}

void Executor::add(task::Periodic &task) {
  for(auto& t: tasks) {
    if (&task.getTask() == &t.getTask()) log.error() << "periodic '" << task.getName() << "' is added twice to the executor";
  }
  tasks.push_back(task);
}

void Executor::add(control::TimeDomain &td) {
  task::Periodic task(td.getName(), td.getPeriod(), td, td.getRealtime());
  for(auto& t: tasks) {
    if (&td == &t.getTask()) log.error() << "periodic of time domain '" << td.getName() << "' is added twice to the executor";
  }
  tasks.push_back(task);
}

void Executor::prefault_stack() {
  unsigned char dummy[8*1024] = {};
    (void)dummy;
}

bool Executor::lock_memory() {
  return (mlockall(MCL_CURRENT | MCL_FUTURE) != -1);
}

bool Executor::set_priority(int nice) {
  struct sched_param schedulingParam;
  schedulingParam.sched_priority = (Executor::basePriority - nice);
  return (sched_setscheduler(0, SCHED_FIFO, &schedulingParam) != -1);
}

void Executor::assignPriorities() {
  std::vector<task::Periodic*> priorityAssignments;

  // add task to list of priority assignments
  traverse(tasks, [&priorityAssignments] (task::Periodic *task) {
    priorityAssignments.push_back(task);
  });

  // sort list of priority assignments
  std::sort(priorityAssignments.begin(), priorityAssignments.end(), [] (task::Periodic *a, task::Periodic *b) -> bool {
    if (a->getRealtime() == b->getRealtime())
      return (a->getPeriod() < b->getPeriod());
    else
      return a->getRealtime();
  });

  // assign priorities
  int nice = 1;
  for (auto t: priorityAssignments) {
    if (t->getRealtime()) {
      t->setNice(nice++);
    }
  }
}

void Executor::stop() {
  auto &instance = Executor::instance();
  instance.running = false;
#ifdef USE_ETHERCAT
  if(instance.etherCATStack) instance.etherCATStack->stop();
#endif
#ifdef USE_ROS2
  rclcpp::shutdown();
  if (instance.subscriberThread != nullptr) {
    instance.subscriberThread->join();
  }
  instance.handleTopic(); // release lock to stop executor
#endif
}

#if defined USE_ROS || defined USE_ROS2
void Executor::syncWithRosTime() {
  syncWithRosTimeSet = true;
  eeros::System::useRosTime();
}
#endif

#ifdef USE_ROS
void Executor::syncWithRosTopic(ros::CallbackQueue* syncRosCallbackQueue) {
  log.warn() << "sync executor with gazebo";
  syncWithRosTopicSet = true;
  this->syncRosCallbackQueue = syncRosCallbackQueue;
}
#endif

#ifdef USE_ROS2
rclcpp::CallbackGroup::SharedPtr Executor::registerSubscriber(rclcpp::Node::SharedPtr node, const bool sync) {
  if (sync) {
    syncWithRosTopicSet = true;
    log.warn() << "sync executor with gazebo";
  }
  if (subscriberExecutor == nullptr) {
    subscriberExecutor = rclcpp::executors::MultiThreadedExecutor::make_shared();
  }
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  subscriberExecutor->add_callback_group(callback_group, node->get_node_base_interface());
  return callback_group;
}

void Executor::handleTopic() {
  cv.notify_one();
}
#endif

void Executor::run() {
  log.trace() << "starting executor with base period " << period << " sec and priority " << (int)(basePriority) << " (thread " << getpid() << ":" << syscall(SYS_gettid) << ")";
  if (period == 0.0) throw std::runtime_error("period of executor not set");
  log.trace() << "assigning priorities";
  assignPriorities();
  Runnable *mainTask = nullptr;
  if (this->mainTask != nullptr) {
    mainTask = &this->mainTask->getTask();
    log.trace() << "setting '" << this->mainTask->getName() << "' as main task";
  }
  std::vector<std::shared_ptr<TaskThread>> threads; // smart pointer used because async objects must not be copied
  task::HarmonicTaskList taskList;
  task::Periodic executorTask("executor", period, this, true);
  counter.monitors = this->mainTask->monitors;
  createThreads(log, tasks, executorTask, threads, taskList);
  using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
  std::this_thread::sleep_for(seconds(1)); // wait 1 sec to allow threads to be created
  if (!set_priority(0))
    log.error() << "could not set realtime priority";
  prefault_stack();
  if (!lock_memory())
    log.error() << "could not lock memory in RAM";

#ifdef USE_ROS2
  // starts spinning ROS2 subscribers in an own thread
  if (subscriberExecutor != nullptr) {
    subscriberThread = std::make_shared<std::thread>([this]() {
      log.info() << "Starting ROS executor for handling ROS subscriptions";
      subscriberExecutor->spin();
    });
  }
#endif
  running = true;

#ifdef USE_ETHERCAT
  if (etherCATStack) {
    log.trace() << "starting execution synched to etcherCAT stack";
    if (syncWithRosTimeIsSet) log.error() << "Can't use both etherCAT and RosTime to sync executor";
    if (syncWithRosTopicIsSet) log.error() << "Can't use both etherCAT and RosTopic to sync executor";
    while (running) {
      etherCATStack->sync();
      counter.tick();
      taskList.run();
      if (mainTask != nullptr)
        mainTask->run();
      counter.tock();
    }
  } else
#else
#if defined USE_ROS || defined USE_ROS2
  if (syncWithRosTimeSet) {
    log.trace() << "starting execution synched to rosTime";
    if (syncWithEtherCatStackSet) log.error() << "Can't use both RosTime and etherCAT to sync executor";
    if (syncWithRosTopicSet) log.error() << "Can't use both RosTime and RosTopic to sync executor";
    uint64_t periodNsec = static_cast<uint64_t>(period * 1.0e9);
    uint64_t nextCycle = eeros::System::getTimeNs() + periodNsec;
    while (running) {
      // spin and wait for next execution time to match ROS time
      while (eeros::System::getTimeNs() < nextCycle && running) usleep(10);
      counter.tick();
      taskList.run();
      if (mainTask != nullptr)
        mainTask->run();
      counter.tock();
      nextCycle += periodNsec;
    }
  }
#endif //(USE_ROS || USE_ROS2)
#ifdef USE_ROS
  else if (syncWithRosTopicSet) {
    log.trace() << "starting execution synched to gazebo";
    if (syncWithRosTimeSet) log.error() << "Can't use both RosTopic and RosTime to sync executor";
    if (syncWithEtherCatStackSet) log.error() << "Can't use both RosTopic and etherCAT to sync executor";
    auto timeOld = ros::Time::now();
    auto timeNew = ros::Time::now();
    static bool first = true;
    while (running) {
      if (first) {
        while (timeOld == timeNew  && running) {	// waits for new rosTime beeing published
          usleep(10);
          timeNew = ros::Time::now();
        }
        first = false;
        timeOld = timeNew;
      }
      while (syncRosCallbackQueue->isEmpty() && running) usleep(10);	//waits for new message;
      while (timeOld == timeNew  && running) {		// waits for new rosTime beeing published
        usleep(10);
        timeNew = ros::Time::now();
      }
      timeOld = timeNew;
      syncRosCallbackQueue->callAvailable();
      counter.tick();
      taskList.run();
      if (mainTask != nullptr)
        mainTask->run();
      counter.tock();
    }
  } else
#endif //(USE_ROS)
#ifdef USE_ROS2
    else if (syncWithRosTopicSet) {
      log.trace() << "starting execution synched to gazebo";
      if (syncWithRosTimeSet) log.error() << "Can't use both RosTopic and RosTime to sync executor";
      if (syncWithEtherCatStackSet) log.error() << "Can't use both RosTopic and etherCAT to sync executor";
      while (running) {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk);
        counter.tick();
        taskList.run();
        if (mainTask != nullptr)
          mainTask->run();
        counter.tock();
      }
    } else
#endif //(USE_ROS2)
  {
    log.trace() << "starting periodic execution";
    // use system time as a start and wait for regular intervals
    auto nextCycle = std::chrono::steady_clock::now() + seconds(period);
    while (running) {
      std::this_thread::sleep_until(nextCycle);
      counter.tick();
      taskList.run();
      if (mainTask != nullptr)
        mainTask->run();
      counter.tock();
      nextCycle += seconds(period);
    }
  }
#endif //(USE_ETHERCAT)

  log.trace() << "stopping all threads";
  for (auto &t: threads) t->async.stop();
  log.trace() << "joining all threads";
  for (auto &t: threads) t->async.join();
  log.trace() << "exiting executor " << " (thread " << getpid() << ":" << syscall(SYS_gettid) << ")";
}
