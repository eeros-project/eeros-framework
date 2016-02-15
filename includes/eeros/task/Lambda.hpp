#ifndef ORG_EEROS_TASK_LAMBDA_HPP_
#define ORG_EEROS_TASK_LAMBDA_HPP_

#include <functional>
#include <eeros/core/Runnable.hpp>

namespace eeros {
	namespace task {

		class Lambda : public Runnable {
		public:
			Lambda();
			Lambda(std::function<void()> f);
			virtual void run();
		private:
			std::function<void()> f;
		};

	}
}

#endif // ORG_EEROS_TASK_LAMBDA_HPP_
