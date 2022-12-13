#include <eeros/task/HarmonicTaskList.hpp>

using namespace eeros::task;

void HarmonicTaskList::run() {
  for (auto &task: tasks) {
    task.run();
  }
}

void HarmonicTaskList::add(Runnable *task, int n) {
  tasks.push_back(Harmonic(task, n));
}

void HarmonicTaskList::add(Runnable &task, int n) {
  tasks.push_back(Harmonic(task, n));
}
