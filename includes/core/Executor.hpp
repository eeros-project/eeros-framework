#ifndef ORG_EEROS_CORE_EXECUTOR_HPP_
#define ORG_EEROS_CORE_EXECUTOR_HPP_

#include <list>

class Executor
{
public:
	Executor(double period);
	virtual int getThreadId();
	virtual int addRunMethod(void (*action)());
	virtual void start();
	virtual bool isTerminated();
private:
  std::list<void (*)()> runMethods;
  int threadId;
};

#endif // ORG_EEROS_CORE_EXECUTOR_HPP_
