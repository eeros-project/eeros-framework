#include <eeros/core/ExecutorService.hpp>

#include <cstdlib>
#include <iostream>
#include <ostream>
#include <windows.h>
#include <time.h>
#include <string.h>

#define NSEC_PER_SEC (1000000000) /* The number of nsecs per sec. */
#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is guaranteed safe to access without faulting */

DWORD WINAPI threadAction(LPVOID ptr) {
	Executor* e = (Executor*) ptr;
	
	int interval = (int)(e->period * NSEC_PER_SEC); /* s -> ns */

	while(e->status != Executor::kStop) {
		// TODO wait for timer
		e->run();
		// TODO calculate next shot
	}
	std::cout << "Thread finished" << std::endl;
	e->status = Executor::kStopped;
	return 0;
}

DWORD dwThreads[MAX_NOF_THREADS] = {0, 0, 0, 0, 0, 0, 0, 0};
HANDLE hThreads[MAX_NOF_THREADS];


int ExecutorService::nofThreads = 0;

int ExecutorService::createNewThread(Executor* e) {
	int threadId = nofThreads;
	hThreads[threadId] = CreateThread(NULL, MAX_SAFE_STACK, threadAction, (LPVOID)e, 0, &dwThreads[threadId]);
	std::cout << "Thread[" << threadId << "] created with thread identifier " << dwThreads[threadId] << std::endl;
	return threadId;
}
