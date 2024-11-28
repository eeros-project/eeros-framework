#ifndef ORG_EEROS_CORE_SEMAPHORE_HPP_
#define ORG_EEROS_CORE_SEMAPHORE_HPP_

#include <mutex>
#include <chrono>
#include <condition_variable>
#include <atomic>

namespace eeros {

	class Semaphore
	{
	public:
		Semaphore(int value = 0);
		void wait();
		bool wait(double timeout_sec);
		void post();
		bool isFree() const noexcept;
	private:
		std::mutex mutex;
		std::condition_variable condvar;
		std::atomic_int counter;
	};

};

#endif /* ORG_EEROS_CORE_SEMAPHORE_HPP_ */
