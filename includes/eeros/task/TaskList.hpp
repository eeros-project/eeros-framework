#ifndef ORG_EEROS_TASK_TASKLIST_HPP_
#define ORG_EEROS_TASK_TASKLIST_HPP_

#include <vector>
#include <eeros/core/Runnable.hpp>

namespace eeros {
	namespace task {

		class TaskList : public Runnable {
		public:
			virtual void run();
			virtual void add(Runnable *t);
			virtual void add(Runnable &t);
			std::vector<Runnable*> tasks;
		};

	}
}

#endif // ORG_EEROS_TASK_TASKLIST_HPP_
