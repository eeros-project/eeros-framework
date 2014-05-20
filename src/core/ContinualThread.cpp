#include <eeros/core/ContinualThread.hpp>

#include <sched.h>
#include <time.h>

using namespace eeros;

ContinualThread::ContinualThread() : s(running),
	Thread([&]() {
		while(s != stopping) {
			if(s != paused) this->run();
		}
		s = stopped;
	}) {
}

ContinualThread::~ContinualThread() {
	stop();
	join();
}

ContinualThread::status ContinualThread::getStatus() const {
	return s;
}

void ContinualThread::start() {
	s = running;
}

void ContinualThread::pause() {
	s = paused;
}

void ContinualThread::stop() {
	if(s != stopped) s = stopping;
}
