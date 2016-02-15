#include <eeros/core/Semaphore.hpp>

using namespace eeros;


Semaphore::Semaphore(int value) :
	counter(value) { }

void Semaphore::wait() {
	std::unique_lock<std::mutex> lock(mutex);
	condvar.wait(lock, [this] { return (counter > 0); });
	counter--;
}

bool Semaphore::wait(double timeout_sec) {
	auto timeout = std::chrono::duration<double, std::chrono::seconds::period>(timeout_sec);
	std::unique_lock<std::mutex> lock(mutex);
	bool result = condvar.wait_for(lock, timeout, [this] { return (counter > 0); });
	if (result) counter--;
	return result;
}

void Semaphore::post() {
	{
		std::unique_lock<std::mutex> lock(mutex);
		counter++;
	}
	condvar.notify_one();
}
