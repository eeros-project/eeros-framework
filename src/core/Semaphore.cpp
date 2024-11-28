#include <eeros/core/Semaphore.hpp>

using namespace eeros;


Semaphore::Semaphore(int value) :
	counter(value) { }

void Semaphore::wait() {
	std::unique_lock<std::mutex> lock(mutex);
	condvar.wait(lock, [this] { return (counter.load(std::memory_order_relaxed) > 0); });
	counter.fetch_sub(1, std::memory_order_relaxed);
}

bool Semaphore::wait(double timeout_sec) {
	auto timeout = std::chrono::duration<double, std::chrono::seconds::period>(timeout_sec);
	std::unique_lock<std::mutex> lock(mutex);
	bool result = condvar.wait_for(lock, timeout, [this] { return (counter.load(std::memory_order_relaxed) > 0); });
	if (result) counter.fetch_sub(1, std::memory_order_relaxed);
	return result;
}

void Semaphore::post() {
	{
		std::unique_lock<std::mutex> lock(mutex);
		counter.fetch_add(1, std::memory_order_relaxed);
	}
	condvar.notify_one();
}

bool Semaphore::isFree() const noexcept {
	return counter.load(std::memory_order_relaxed) == 0;
}
