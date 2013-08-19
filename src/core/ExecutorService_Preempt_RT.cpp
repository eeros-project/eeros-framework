#include <eeros/core/ExecutorService.hpp>

#include <cstdlib>
#include <iostream>
#include <ostream>
#include <pthread.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>

#define RT_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50 as the priority of kernel tasklets and interrupt handler by default */
#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is guaranteed safe to access without faulting */
#define NSEC_PER_SEC (1000000000) /* The number of nsecs per sec. */

void stack_prefault(void) {
	unsigned char dummy[MAX_SAFE_STACK];
	memset(dummy, 0, MAX_SAFE_STACK);
	return;
}

int ExecutorService::nofThreads = 0;
pthread_t ExecutorService::threads[MAX_NOF_THREADS] = {0, 0, 0, 0, 0, 0, 0, 0};

int ExecutorService::createNewThread(Executor* e) {
	int threadId = nofThreads;
	int ret;
	ret = pthread_create(&ExecutorService::threads[ExecutorService::nofThreads++], NULL, threadAction, (void*)e);
	std::cout << "Real time thread (#" << threadId << ") created with return value " << ret << std::endl;
	return threadId;
}

void* ExecutorService::threadAction(void* ptr) {
	Executor* e = (Executor*) ptr;
	struct timespec t;
	struct sched_param param;
	int interval = (int)(e->getPeriod() * NSEC_PER_SEC); /* s -> ns */

	param.sched_priority = RT_PRIORITY;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
		std::cerr << "sched_setscheduler failed" << std::endl;
		exit(-1);
	}
    
	/* Lock memory */
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		std::cerr << "mlockall failed" << std::endl;
		exit(-2);
	}
    
	/* Pre-fault our stack */
	stack_prefault();
	clock_gettime(CLOCK_MONOTONIC ,&t);
	/* start after one second */
	//t.tv_sec++;

	while(e->getStatus() != kStop) {
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
		e->run();
		t.tv_nsec += interval;
		while (t.tv_nsec >= NSEC_PER_SEC) {
			t.tv_nsec -= NSEC_PER_SEC;
			t.tv_sec++;
		}
	}
	munlockall();
	std::cout << "Thread finished" << std::endl;
	e->status = kStopped;
}

void ExecutorService::waitForSequenceEnd(Executor* waitExecutor) {
	// TODO
}