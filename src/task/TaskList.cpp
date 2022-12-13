#include <eeros/task/TaskList.hpp>

using namespace eeros::task;

void TaskList::run() {
  for (auto t: tasks) {
    t->run();
  }
}

void TaskList::add(Runnable *t) {
  tasks.push_back(t);
}
void TaskList::add(Runnable &t) {
  tasks.push_back(&t);
}
