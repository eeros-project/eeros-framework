#ifndef ORG_EEROS_TASK_ASYNC_HPP_
#define ORG_EEROS_TASK_ASYNC_HPP_

#include <thread>
#include <atomic>

#include <eeros/task/Periodic.hpp>
#include <eeros/core/Semaphore.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/TimeSource.hpp>

namespace eeros {
namespace task {

class Async : public Runnable {
 public:
  Async(Runnable &task, double period, bool realtime = false, int nice = 0);
  Async(Runnable *task, double period, bool realtime = false, int nice = 0);
  virtual ~Async();
  virtual void run();
  void stop();
  void join();
  bool cycleComplete();

  PeriodicCounter counter;

 private:
  void run_thread();
  Runnable &task;
  bool realtime;
  int nice;
  double period;
  Semaphore semaphore;
  std::thread thread;
  std::atomic<bool> finished;
  // std::atomic<uint32_t> runCycle{0};
  // std::atomic<uint32_t> checkCycle{1};
  enum class TaskState { Idle, Running, WaitingToRun};
  std::atomic<TaskState> state{TaskState::Idle};
  logger::Logger log;
};

}
}

#endif // ORG_EEROS_TASK_ASYNC_HPP_
