#include "core/Executor.hpp"
#include "core/ExecutorService.hpp"

Executor::Executor(double period)
{
	this->period = period;
}

int Executor::getThreadId()
{
	return this->threadId;
}

double Executor::getPeriod()
{
	return this->period;
}

int Executor::getStatus()
{
	return this->status;
}

void Executor::addRunMethod(void (*run)())
{
	runMethods.push_back(run);
}

void Executor::start()
{
	threadId = ExecutorService::createNewThread(this);
}

bool Executor::isTerminated()
{
	return status == kStopped;
}
void Executor::stop()
{
	this->status = kStop;
}

void Executor::run()
{
	for(std::list<void (*)()>::iterator i = runMethods.begin(); i != runMethods.end(); i++)
	{
		(*i)();
	}
}