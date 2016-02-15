#include <eeros/task/Harmonic.hpp>

using namespace eeros::task;


Harmonic::Harmonic(Runnable &task, int n) :
	n(n), k(0), task(&task) { }

Harmonic::Harmonic(Runnable *task, int n) :
	n(n), k(0), task(task) { }

eeros::Runnable * Harmonic::getTask() {
	return task;
}

void Harmonic::run() {
	if (++k >= n) {
		task->run();
		k = 0;
	}
}
