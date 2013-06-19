#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <list>
#include <eeros/core/Runnable.hpp>
#include <eeros/core/ExecutorService.hpp>

enum { kRunning = 0, kStop = 1, kStopped = 2 };

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
	
//#ifndef WINDOWS		// TODO Check why the visual studio compiler the friend declaration above not understands. Remove this pre processor macro.
	private:		// PARV has no problem with private VS 2010, I have no WINDOWS define but _WINDOWS!
//#endif
	virtual void run();
	
	int status;
	double period;
	std::list<Runnable*> runnables;
	int threadId;
};

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
