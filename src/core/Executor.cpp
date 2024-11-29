#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/task/Async.hpp>
#include <eeros/task/HarmonicTaskList.hpp>
#include <eeros/task/Lambda.hpp>
#include <memory>
#include <sched.h>
#include <signal.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>
#include <vector>
#ifdef USE_ROS
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#endif

using namespace eeros;
using namespace std::chrono;

namespace {

using Logger = logger::Logger;

struct TaskThread {
  TaskThread(double period, task::Periodic &task, task::HarmonicTaskList tasks)
      : taskList(tasks), async(taskList, period, task.getRealtime(), task.getNice()), period(period) {
    async.counter.setPeriod(period);
    async.counter.monitors = task.monitors;
  }
  task::HarmonicTaskList taskList;
  task::Async async;
  double period;
  int deadlineCycle = 0;
};

template <typename F>
void traverse(std::vector<task::Periodic> &tasks, F func) {
  for (auto &t : tasks) {
    func(&t);
    traverse(t.before, func);
    traverse(t.after, func);
  }
}

void createThread(Logger &log, task::Periodic &task, task::Periodic &baseTask,
                  std::vector<std::shared_ptr<TaskThread>> &threads,
                  std::vector<task::Harmonic> &output);

void createThreads(Logger &log, std::vector<task::Periodic> &tasks,
                   task::Periodic &baseTask,
                   std::vector<std::shared_ptr<TaskThread>> &threads,
                   task::HarmonicTaskList &output) {
  for (task::Periodic &t : tasks) {
    createThread(log, t, baseTask, threads, output.tasks);
  }
}

void createThread(Logger &log, task::Periodic &task, task::Periodic &baseTask,
                  std::vector<std::shared_ptr<TaskThread>> &threads,
                  std::vector<task::Harmonic> &output) {
  int k = static_cast<int>(task.getPeriod() / baseTask.getPeriod());
  double actualPeriod = k * baseTask.getPeriod();
  double deviation =
      std::abs(task.getPeriod() - actualPeriod) / task.getPeriod();
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
                << "' with period " << actualPeriod << " sec (k = " << k
                << ") and priority "
                << ((int)(Executor::basePriority)-task.getNice())
                << " based on '" << baseTask.getName() << "'";
  else
    log.trace() << "creating harmonic task '" << task.getName()
                << "' with period " << actualPeriod << " sec (k = " << k << ")"
                << " based on '" << baseTask.getName() << "'";

  if (deviation > 0.01)
    throw std::runtime_error("period deviation too high");

  if (task.getRealtime() && task.getNice() <= 0)
    throw std::runtime_error("priority not set");

  if (taskList.tasks.size() == 0)
    throw std::runtime_error("no task to execute");

  threads.push_back(std::make_shared<TaskThread>(actualPeriod, task, taskList));
  output.emplace_back(threads.back()->async, k);
}
} // namespace

Executor::Executor()
    : period(0), mainTask(nullptr), syncWithEtherCatStackSet(false),
      syncWithRosTimeSet(false), syncWithRosTopicSet(false),
      log(logger::Logger::getLogger('E')) {}

Executor::~Executor() {}

Executor &Executor::instance() {
  static Executor executor;
  return executor;
}

#ifdef USE_ETHERCAT
void Executor::syncWithEtherCATSTack(
    ecmasterlib::EcMasterlibMain *etherCATStack) {
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
  task::Periodic *task =
      new task::Periodic("safety system", ss.getPeriod(), ss, true);
  setMainTask(*task);
}

void Executor::setExecutorPeriod(double period) {
  if (this->mainTask != nullptr)
    throw std::runtime_error(
        "set the executor period only in case you dont't have a main task");
  task::Periodic *task =
      new task::Periodic("default main task", period, new task::Lambda(), true);
  setMainTask(*task);
}

task::Periodic *Executor::getMainTask() { return mainTask; }

void Executor::add(task::Periodic &task) {
  for (auto &t : tasks) {
    if (&task.getTask() == &t.getTask())
      log.error() << "periodic '" << task.getName()
                  << "' is added twice to the executor";
  }
  tasks.push_back(task);
}

void Executor::add(control::TimeDomain &td) {
  task::Periodic task(td.getName(), td.getPeriod(), td, td.getRealtime());
  for (auto &t : tasks) {
    if (&td == &t.getTask())
      log.error() << "periodic of time domain '" << td.getName()
                  << "' is added twice to the executor";
  }
  tasks.push_back(task);
}

void Executor::prefault_stack() {
  unsigned char dummy[8 * 1024] = {};
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
  std::vector<task::Periodic *> priorityAssignments;

  // add task to list of priority assignments
  traverse(tasks, [&priorityAssignments](task::Periodic *task) {
    priorityAssignments.push_back(task);
  });

  // sort list of priority assignments
  std::sort(priorityAssignments.begin(), priorityAssignments.end(),
            [](task::Periodic *a, task::Periodic *b) -> bool {
              if (a->getRealtime() == b->getRealtime())
                return (a->getPeriod() < b->getPeriod());
              else
                return a->getRealtime();
            });

  // assign priorities
  int nice = 1;
  for (auto t : priorityAssignments) {
    if (t->getRealtime()) {
      t->setNice(nice++);
    }
  }
}

void Executor::stop() {
  auto &instance = Executor::instance();
  instance.running = false;
#ifdef USE_ETHERCAT
  if (instance.etherCATStack)
    instance.etherCATStack->stop();
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
void Executor::syncWithRosTopic(ros::CallbackQueue *syncRosCallbackQueue) {
  log.warn() << "sync executor with gazebo";
  syncWithRosTopicSet = true;
  this->syncRosCallbackQueue = syncRosCallbackQueue;
}
#endif

#ifdef USE_ROS2
rclcpp::CallbackGroup::SharedPtr
Executor::registerSubscriber(rclcpp::Node::SharedPtr node, const bool sync) {
  if (sync) {
    syncWithRosTopicSet = true;
    log.warn() << "sync executor with gazebo";
  }
  if (subscriberExecutor == nullptr) {
    subscriberExecutor =
        rclcpp::executors::MultiThreadedExecutor::make_shared();
  }
  auto callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  subscriberExecutor->add_callback_group(callback_group,
                                         node->get_node_base_interface());
  return callback_group;
}

void Executor::handleTopic() { cv.notify_one(); }
#endif

void Executor::run() {
  log.trace() << "starting executor with base period " << period
              << " sec and priority " << (int)(basePriority) << " (thread "
              << getpid() << ":" << syscall(SYS_gettid) << ")";
  if (period == 0.0&& sync->getPeriod().count() != 0) {
    if(sync)
      period = sync->getPeriod().count();
    else
      throw std::runtime_error("period of executor not set");
  }
  log.trace() << "assigning priorities";
  assignPriorities();
  Runnable *mainTask = nullptr;
  if (this->mainTask != nullptr) {
    mainTask = &this->mainTask->getTask();
    log.trace() << "setting '" << this->mainTask->getName() << "' as main task";
  }
  std::vector<std::shared_ptr<TaskThread>>
      threads; // smart pointer used because async objects must not be copied
  task::HarmonicTaskList taskList;
  task::Periodic executorTask("executor", period, this, true);
  if(this->mainTask)
    counter.monitors = this->mainTask->monitors;
  createThreads(log, tasks, executorTask, threads, taskList);
  using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
  std::this_thread::sleep_for(
      seconds(1)); // wait 1 sec to allow threads to be created
  if (!set_priority(0))
    log.error() << "could not set realtime priority";
  prefault_stack();
  if (!lock_memory())
    log.error() << "could not lock memory in RAM";
  if (!sync)
    sync = std::make_shared<core::SystemTime>(period);

  running = true;

  log.trace() << "starting periodic execution";
  sync->start();
  while (running) {
    counter.tick();
    time.currentCycle.fetch_add(1, std::memory_order::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order::memory_order_acquire);
    taskList.run();
    if (mainTask != nullptr)
      mainTask->run();

    // make sure we wait for all async tasks to have completed the cycle before we sync memory operations
    // also makes sure we stay in sync for TimeSources like ManualClock that don't sleep between cycles
    while (!std::all_of(threads.begin(), threads.end(), [](auto t){return t->async.cycleComplete();}));
    std::atomic_thread_fence(std::memory_order::memory_order_release);
    counter.tock();
    sync->sync();
  }

  log.trace() << "stopping all threads";
  for (auto &t : threads)
    t->async.stop();
  log.trace() << "joining all threads";
  for (auto &t : threads)
    t->async.join();
  log.trace() << "exiting executor " << " (thread " << getpid() << ":"
              << syscall(SYS_gettid) << ")";
}
