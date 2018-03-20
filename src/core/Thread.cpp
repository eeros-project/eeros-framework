#include <eeros/core/Thread.hpp>
#include <sstream>
#include <sched.h>

using namespace eeros;

Thread::Thread() : t([&]() {
	std::string id = getId();
	struct sched_param schedulingParam;
	schedulingParam.sched_priority = 5;
	if (sched_setscheduler(0, SCHED_FIFO, &schedulingParam) != 0) log.error() << "could not set realtime priority";
	sched_getparam(0, &schedulingParam);
	log.trace() << "Thread '" << id << "' with prio=" << schedulingParam.sched_priority << " started.";
	this->run();
	log.trace() << "Thread '" << id << "' finished.";
}) { }

Thread::Thread(std::function<void ()> t) : t(t) { }

Thread::~Thread() {join();}

std::string Thread::getId() const {
	std::ostringstream s;
	s << t.get_id();
	return s.str();
}

void Thread::join() {
	if(t.joinable()) t.join();
}

void Thread::run() {log.warn() << "base class";}
