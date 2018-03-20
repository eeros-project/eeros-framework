#include <thread>

#include <eeros/task/Periodic.hpp>

using namespace eeros::task;


Periodic::Periodic(const char *name, double period, Runnable &task, bool realtime, int nice) :
	name(name), period(period), task(&task), realtime(realtime), nice(nice) { }

Periodic::Periodic(const char *name, double period, Runnable *task, bool realtime, int nice) :
	name(name), period(period), task(task), realtime(realtime), nice(nice) { }

void Periodic::addDefaultMonitor(double tolerance) {
	PeriodicCounter::addDefaultMonitor(monitors, period, tolerance);
}

std::string Periodic::getName() {
	return name;
}

double Periodic::getPeriod() {
	return period;
}

eeros::Runnable& Periodic::getTask() {
	return *task;
}

bool Periodic::getRealtime() {
	return realtime;
}

int Periodic::getNice() {
	return nice;
}

void Periodic::setNice(int value) {
	nice = value;
}

