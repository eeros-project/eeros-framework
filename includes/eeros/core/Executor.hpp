#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <list>
#include <eeros/core/Runnable.hpp>
#include <eeros/core/ExecutorService.hpp>

namespace eeros {

	enum { kRunning = 0, kStop = 1, kStopped = 2 };

	class Executor {
		friend class ExecutorService;

	public:
		Executor(double period);
		virtual ~Executor();
		virtual int getThreadId();
		virtual double getPeriod();
		virtual int getStatus();
		virtual void addRunnable(Runnable* runnable);
		virtual void addRunnable(Runnable& runnable);
		virtual void start();
		virtual bool isTerminated();
		virtual void stop();
		virtual void join();
		
	protected:
		virtual void run();
		
	private:
		int status;
		double period;
		std::list<Runnable*> runnables;
		int threadId;
	};
};

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
