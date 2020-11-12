#include <eeros/core/Thread.hpp>
#include <sstream>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>

using namespace eeros;

Thread::Thread(int priority) : log(logger::Logger::getLogger('T')), t([&,priority]() {
	struct sched_param schedulingParam;
	schedulingParam.sched_priority = priority;
	if (sched_setscheduler(0, SCHED_FIFO, &schedulingParam) != 0) /*log.error() << "could not set realtime priority"*/;
	sched_getparam(0, &schedulingParam);
	log.trace() << "thread " << getpid() << ":" << syscall(SYS_gettid) << " with priority " << schedulingParam.sched_priority << " started.";
	this->run();
	log.trace() << "thread " << getpid() << ":" << syscall(SYS_gettid) << " finished.";
}) { }

Thread::Thread(std::function<void ()> t) : log(logger::Logger::getLogger('T')), t(t) { }

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
