#include <eeros/core/ExecutorService.hpp>

#include <cstdlib>
#include <pthread.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>

#define NSEC_PER_SEC (1000000000) /* The number of nsecs per sec. */

using namespace eeros;
using namespace eeros::logger;

int ExecutorService::nofThreads = 0;
pthread_t ExecutorService::threads[MAX_NOF_THREADS] = {0, 0, 0, 0, 0, 0, 0, 0}; // TODO
Logger<LogWriter> ExecutorService::log;

int ExecutorService::createNewThread(Executor* e) {
	int threadId = ExecutorService::nofThreads++;
	int ret;
	ret = pthread_create(&ExecutorService::threads[threadId], NULL, ExecutorService::threadAction, (void*)e);
	log.info() << "Thread #" << threadId << " created with return value '" << ret << "'.";
	return threadId;
}

/**** TODO check if simulation or not! ****/

void* ExecutorService::threadAction(void* ptr) {
	Executor* e = (Executor*)ptr;
	struct timespec t;

	int interval = (int)(e->period * NSEC_PER_SEC); /* s -> ns */

	clock_gettime(CLOCK_MONOTONIC ,&t);

	while(e->status != kStop) {
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
		e->run();
		if(interval > 0) { // calculate next shot
			t.tv_nsec += interval;
			while (t.tv_nsec >= NSEC_PER_SEC) {
				t.tv_nsec -= NSEC_PER_SEC;
				t.tv_sec++;
			}
		}
		else { // oneshot -> terminate
			e->stop();
		}
	}
	log.info() << "Thread '" << e->threadId << "'finished";
	e->status = kStopped;
}