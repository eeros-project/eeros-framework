#ifndef ORG_EEROS_TASK_HARMONICTASKLIST_HPP_
#define ORG_EEROS_TASK_HARMONICTASKLIST_HPP_

#include <vector>
#include <eeros/core/Runnable.hpp>
#include <eeros/task/Harmonic.hpp>

namespace eeros {
	namespace task {

		class HarmonicTaskList : public Runnable {
		public:
			virtual void run();
			virtual void add(Runnable *t, int n = 1);
			virtual void add(Runnable &t, int n = 1);
			std::vector<Harmonic> tasks;
		};

	}
}

#endif // ORG_EEROS_TASK_HARMONICTASKLIST_HPP_
