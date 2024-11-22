#ifndef ORG_EEROS_TASK_ASYNC_HPP_
#define ORG_EEROS_TASK_ASYNC_HPP_

#include <thread>
#include <atomic>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/Semaphore.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/logger/Logger.hpp>

namespace eeros {
namespace task {

class Async : public Runnable {
 public:
  Async(Runnable &task, bool realtime = false, int nice = 0);
  Async(Runnable *task, bool realtime = false, int nice = 0);
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
  Semaphore semaphore;
  std::thread thread;
  std::atomic<bool> finished;
  std::atomic<uint32_t> runCycle{0};
  std::atomic<uint32_t> checkCycle{1};
  logger::Logger log;
};

}
}

#endif // ORG_EEROS_TASK_ASYNC_HPP_
