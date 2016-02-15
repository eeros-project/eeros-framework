#ifndef ORG_EEROS_TASK_HARMONIC_HPP_
#define ORG_EEROS_TASK_HARMONIC_HPP_

#include <eeros/core/Runnable.hpp>

namespace eeros {
	namespace task {

		class Harmonic : public Runnable {
		public:
			Harmonic(Runnable &task, int n = 1);
			Harmonic(Runnable *task, int n = 1);
			Runnable *getTask();
			virtual void run();
		private:
			int n, k;
			Runnable *task;
		};

	}
}

#endif // ORG_EEROS_TASK_HARMONIC_HPP_
