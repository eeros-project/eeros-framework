#include <stdexcept>
#include <sys/types.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <eeros/task/Async.hpp>
#include <eeros/core/Executor.hpp>

using namespace eeros::task;
using namespace eeros::logger;

Async::Async(Runnable &task, double period, bool realtime , int nice)
    : task(task), realtime(realtime), nice(nice), period(period), thread(&Async::run_thread, this),
      finished(false), log(Logger::getLogger('A')) { }

Async::Async(Runnable *task, double period, bool realtime , int nice)
    : task(*task), realtime(realtime), nice(nice), period(period), thread(&Async::run_thread, this),
      finished(false), log(Logger::getLogger('A')){ }

Async::~Async() {
  stop();
  join();
}

void Async::run() {
  state.store(TaskState::WaitingToRun, std::memory_order_relaxed);
  semaphore.post();
}

bool Async::cycleComplete()
{
  // auto areWeBehind = [](uint32_t current, uint32_t checked){
  //       auto difference = current > checked ? current - checked : checked - current;
  //       bool assumeOverflow = difference > std::numeric_limits<decltype(current)>::max()/2;
  //       if (assumeOverflow) return current < checked;
  //       else return current > checked;
  // };
  // while(true) {
  //   auto current = runCycle.load(std::memory_order::memory_order_relaxed);
  //   auto checked = checkCycle.load(std::memory_order::memory_order_relaxed);
  //   if (current == checked) { // we are in sync
  //     if (checkCycle.compare_exchange_weak(checked, checked+1, std::memory_order::memory_order_relaxed)) {
  //       return true;
  //     }
  //   } else if (areWeBehind(current, checked)) { // run() is multiple cycles ahead, so the cycle we are checking is complete
  //       if(checkCycle.compare_exchange_weak(checked, current+1)) {
  //         return true;
  //
  //       }
  //
  //   } else
  //   {
  //     return false;
  //   }
  //
  // }

  return state.load(std::memory_order_relaxed) == TaskState::Idle;
}


void Async::stop() {
  finished.store(true, std::memory_order::memory_order_relaxed);
  semaphore.post();
}

void Async::join() {
  if (thread.joinable()) thread.join();
}

void Async::run_thread() {
  const auto pid = getpid();
  const auto tid = syscall(SYS_gettid);

  Executor::prefault_stack();

  if (realtime) {
    int priority = Executor::basePriority - nice;
    log.trace() << "starting realtime thread " << pid << ":" << tid << " with priority " << priority;

    if (!Executor::set_priority(nice))
      log.error() << "could not set realtime priority";

    if (!Executor::lock_memory())
      log.error() << "could not lock memory in RAM";
  }
  else {
    log.trace() << "starting thread " << pid << ":" << tid;
  }

  while (!finished.load(std::memory_order::memory_order_relaxed)) {
    semaphore.wait();
    state.store(TaskState::Running, std::memory_order_relaxed);
    counter.tick();
    std::atomic_thread_fence(std::memory_order::memory_order_acquire);
    task.run();
    std::atomic_thread_fence(std::memory_order::memory_order_release);
    counter.tock();
    // runCycle.fetch_add(1, std::memory_order::memory_order_relaxed);
    auto s = TaskState::Running;
    while(s == TaskState::Running) state.compare_exchange_weak(s, TaskState::Idle, std::memory_order_relaxed);
  }

  log.trace() << "stopping thread " << pid << ":" << tid;
}
