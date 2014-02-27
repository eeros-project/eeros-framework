#ifndef ORG_EEROS_CORE_EXECUTORSERVICE_HPP_
#define ORG_EEROS_CORE_EXECUTORSERVICE_HPP_

#include <eeros/config.hpp>
#include <eeros/core/Executor.hpp>

#if defined(WINDOWS)
#include <windows.h>
#endif

#if defined(LINUX) || defined(PREEMPT_RT) || defined(FREEBSD)
#include <pthread.h>
#endif

#define RUNNING 1
#define MAX_NOF_THREADS 64

namespace eeros {

	class Executor;

	class ExecutorService {

	public:
		static int createNewThread(Executor* e);
		static void waitForSequenceEnd(Executor* waitExecutor);

	private:
		static int nofThreads;
	#if defined(WINDOWS)
		static HANDLE getHandle(int i);
		static DWORD WINAPI threadAction(LPVOID ptr);
		static DWORD dwThreads[];
		static HANDLE hThreads[];
	#endif
	#if defined(LINUX) || defined(PREEMPT_RT) || defined(FREEBSD)
		static void* threadAction(void*);
		static pthread_t threads[];
	#endif
	#if defined(XENOMAI)
		// TODO
	#endif

	#if defined(QNX)
		// TODO
	#endif
	};
};

#endif // ORG_EEROS_CORE_EXECUTORSERVICE_HPP_
