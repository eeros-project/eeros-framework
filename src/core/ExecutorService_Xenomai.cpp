#include <eeros/core/ExecutorService.hpp>

#include <cstdlib>
#include <iostream>
#include <ostream>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>




#define RT_PRIORITY (90) /* we use 90 cause no other idea xD */
#define NSEC_PER_SEC (1000000000) /* The number of nsecs per sec. */

int ExecutorService::nofThreads = 0;
RT_TASK ExecutorService::threads[MAX_NOF_THREADS] = {0,0,0,0,0,0,0,0};

/**** TODO check if simulation or not! ****/
void ExecutorService::threadAction(void* ptr) {
	Executor* e = (Executor*) ptr;
	int interval = (int)(e->period * NSEC_PER_SEC); /* s -> ns */
	rt_task_set_periodic(NULL, TM_NOW,rt_timer_ns2tsc(interval));

	while(e->status != kStop) {
		e->run();
		rt_task_wait_period(NULL);
	}
	std::cout << "Thread finished" << std::endl;
	e->status = kStopped;
}


/**** TODO define task name ****/
int ExecutorService::createNewThread(Executor* e) {
	int threadId = nofThreads;
	int ret;
	char taskName[] =  "eeros task 00";
	taskName[12] = (char) ('0'+nofThreads);
	mlockall(MCL_CURRENT|MCL_FUTURE);
	ret = rt_task_create(&ExecutorService::threads[ExecutorService::nofThreads], taskName, 0, RT_PRIORITY, 0);
	std::cout << "Thread[" << threadId << "] created with return value " << ret << std::endl;
	ret = rt_task_start(&ExecutorService::threads[ExecutorService::nofThreads],ExecutorService::threadAction, (void*)e);
	std::cout << "Thread[" << threadId << "] started with return value " << ret << std::endl;
	ExecutorService::nofThreads++;
	return threadId;
}

void ExecutorService::waitForSequenceEnd(Executor* waitExecutor) {
	// TODO
}
