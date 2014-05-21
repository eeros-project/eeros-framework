#include <eeros/core/PeriodicThread.hpp>

#include <sched.h>
#include <time.h>
#include <sys/mman.h>

#define RT_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50 as the priority of kernel tasklets and interrupt handler by default */
#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is guaranteed safe to access without faulting */

using namespace eeros;

void stack_prefault() {
	unsigned char dummy[MAX_SAFE_STACK] = {};
}

PeriodicThread::PeriodicThread(double period, double delay, bool realtime, status start) : 
	rt(realtime),
	period(period),
	delay(delay),
	s(start),
	Thread([this]() {
		struct timespec time;
		uint64_t period_ns = to_ns(this->period);
		
		clock_gettime(CLOCK_MONOTONIC, &time);
		time.tv_nsec += to_ns(this->delay);
		
		if(this->rt) {
			log.info() << "configuring thread as realtime thread!";
			struct sched_param schedulingParam;
			schedulingParam.sched_priority = RT_PRIORITY;
			if(sched_setscheduler(0, SCHED_FIFO, &schedulingParam) == -1) {
//				log.error() << "Thread '" << getId() << "': failed to set real time scheduler!";
				log.error() << "Thread: failed to set real time scheduler!";
			}
			
			/* Lock memory */
			if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
//				log.error() << "Thread '" << getId() << "': failed to lock memory allocation!";
				log.error() << "Thread: failed to lock memory allocation!";
			}
			
			/* Pre-fault our stack */
			stack_prefault(); // TODO should we use pthread_attr_setstacksize() ?
		}
		
		while(time.tv_nsec >= to_ns(1)) {
			time.tv_nsec -= to_ns(1);
			time.tv_sec++;
		}
		while(s != stopping) {
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time, NULL);
			if(s != paused) this->run();
			time.tv_nsec += period_ns;
			while(time.tv_nsec >= to_ns(1)) {
				time.tv_nsec -= to_ns(1);
				time.tv_sec++;
			}
		}
		s = stopped;
	}) {
}

PeriodicThread::~PeriodicThread() {
	stop();
	join();
}

PeriodicThread::status PeriodicThread::getStatus() const {
	return s;
}

double PeriodicThread::getPeriod() const {
	return period;
}

void PeriodicThread::start() {
	s = running;
}

void PeriodicThread::pause() {
	s = paused;
}

void PeriodicThread::stop() {
	s = stopping;
}
