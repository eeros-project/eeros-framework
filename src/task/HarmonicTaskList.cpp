#include <eeros/task/HarmonicTaskList.hpp>

using namespace eeros::task;


void HarmonicTaskList::run() {
	for (auto &t: tasks)
		t.run();
}

void HarmonicTaskList::add(Runnable *t, int n) {
	tasks.push_back(Harmonic(t, n));
}

void HarmonicTaskList::add(Runnable &t, int n) {
	tasks.push_back(Harmonic(t, n));
}
