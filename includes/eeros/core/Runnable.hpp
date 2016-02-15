#ifndef ORG_EEROS_CORE_RUNNABLE_HPP_
#define ORG_EEROS_CORE_RUNNABLE_HPP_

namespace eeros {

	class Runnable {
	public:
		virtual ~Runnable();
		virtual void run() = 0;
	};

};

#endif // ORG_EEROS_CORE_RUNNABLE_HPP_
