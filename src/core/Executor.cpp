#include <eeros/core/Executor.hpp>

using namespace eeros;

Executor::Executor() : threadId(-1), period(0), status(kStopped) { }

Executor::Executor(double period) : threadId(-1), period(period), status(kStopped) { }

Executor::~Executor() { stop(); }

int Executor::getThreadId() {
	return threadId;
}

double Executor::getPeriod() {
	return period;
}

int Executor::getStatus() {
	return status;
}

void Executor::addRunnable(Runnable* runnable) {
	runnables.push_back(runnable);
}

void Executor::addRunnable(Runnable& runnable) {
	runnables.push_back(&runnable);
}

void Executor::start() {
	status = kRunning;
	threadId = ExecutorService::createNewThread(this);
}

bool Executor::isTerminated() {
	return status == kStopped;
}

void Executor::stop() {
	this->status = kStop;
}

void Executor::join() {
	// TODO
}

void Executor::run() {
	for(auto runnable : runnables) {
		runnable->run();
	}
}
