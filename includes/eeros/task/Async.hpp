#ifndef ORG_EEROS_TASK_ASYNC_HPP_
#define ORG_EEROS_TASK_ASYNC_HPP_

#include <thread>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/Semaphore.hpp>
#include <eeros/core/PeriodicCounter.hpp>

namespace eeros {
	namespace task {

		class Async : public Runnable
		{
		public:
			Async(Runnable &task, bool realtime = false, int nice = 0);
			Async(Runnable *task, bool realtime = false, int nice = 0);
			virtual ~Async();
			virtual void run();
			void stop();
			void join();

			PeriodicCounter counter;

		private:
			void run_thread();
			Runnable &task;
			bool realtime;
			int nice;
			Semaphore semaphore;
			std::thread thread;
                        bool finished;
		};

	}
}

#endif // ORG_EEROS_TASK_ASYNC_HPP_
