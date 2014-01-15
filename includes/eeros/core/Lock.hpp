#ifndef ORG_EEROS_CORE_LOCK_HPP_
#define ORG_EEROS_CORE_LOCK_HPP_

#include <eeros/core/Mutex.hpp>

namespace eeros {

	class Lock {
	public:
		Lock(Mutex& mutex);
		~Lock();
		void lock();
		void unlock();
	private:
		Mutex& mutex;
	};

};

#endif // ORG_EEROS_CORE_LOCK_HPP_
