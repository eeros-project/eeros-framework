#ifndef ORG_EEROS_CORE_EXECUTORSERVICE_HPP_
#define ORG_EEROS_CORE_EXECUTORSERVICE_HPP_

#include "core/Executor.hpp"

#if defined(WINDOWS)
#include <windows.h>
#else
#include <pthread.h>
#endif

#define RUNNING 1
#define MAX_NOF_THREADS 8

class ExecutorService {

public:
	static int createNewThread(Executor*);

private:
	static void stack_prefault(void);
	static int nofThreads;
#if defined(POSIX)
	// TODO
#else
	static void* threadAction(void*);
	static pthread_t threads[];
#endif
};

#endif // ORG_EEROS_CORE_EXECUTORSERVICE_HPP_
