#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <list>

class Executor
{
public:
	Executor(double period);
	virtual int getThreadId();
	virtual double getPeriod();
	virtual int getStatus();
	virtual void addRunMethod(void (*run)());
	virtual void start();
	virtual bool isTerminated();
	virtual void stop();
	
	virtual void run();
	
	static const int kRunning = 0;
	static const int kStop = 1;
	static const int kStopped = 2;
	
private:
	int status;
	double period;
	std::list<void (*)()> runMethods;
	int threadId;
};

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
