#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <list>
#include <eeros/core/Runnable.hpp>
#include <eeros/core/ExecutorService.hpp>

class Executor
{
	friend class ExecutorService;

public:
	Executor(double period);
	virtual int getThreadId();
	virtual double getPeriod();
	virtual int getStatus();
	virtual void addRunnable(Runnable* runnable);
	virtual void start();
	virtual bool isTerminated();
	virtual void stop();

	static const int kRunning = 0;
	static const int kStop = 1;
	static const int kStopped = 2;
	
private:
	virtual void run();
	
	int status;
	double period;
	std::list<Runnable*> runnables;
	int threadId;
};

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
