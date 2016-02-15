#include <stdexcept>
#include <sys/types.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <eeros/task/Async.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/core/Executor.hpp>


namespace {
	long gettid() {
		return syscall(SYS_gettid);
	}
}



using namespace eeros::task;


Async::Async(Runnable &task, bool realtime , int nice) :
	task(task), realtime(realtime), nice(nice), thread(&Async::run_thread, this), finished(false) { }

Async::Async(Runnable *task, bool realtime , int nice) :
	task(*task), realtime(realtime), nice(nice), thread(&Async::run_thread, this), finished(false) { }

Async::~Async() {
	stop();
	join();
}

void Async::run() {
	semaphore.post();
}

void Async::stop() {
	finished = true;
	semaphore.post();
}

void Async::join() {
	if (thread.joinable())
		thread.join();
}

void Async::run_thread() {
	logger::Logger<logger::LogWriter> log('A');

	const auto pid = getpid();
	const auto tid = gettid();

	Executor::prefault_stack();

	if (realtime) {
		int priority = Executor::basePriority - nice;
		log.trace() << "starting realtime thread " << pid << ":" << tid << " with priority " << priority;

		if (!Executor::set_priority(nice))
			log.error() << "could not set realtime priority";

		if (!Executor::lock_memory())
			log.error() << "could not lock memory in RAM";
	}
	else {
		log.trace() << "starting thread " << pid << ":" << tid;
	}

	semaphore.wait();
	while (!finished) {
		counter.tick();
		task.run();
		counter.tock();
		semaphore.wait();
	}

	log.trace() << "stopping thread " << pid << ":" << tid;
}
