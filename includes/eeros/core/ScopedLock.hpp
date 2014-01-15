#ifndef ORG_EEROS_CORE_SCOPEDLOCK_HPP_
#define ORG_EEROS_CORE_SCOPEDLOCK_HPP_

#include <eeros/core/Mutex.hpp>

namespace eeros {

	class ScopedLock {
	public:
		ScopedLock(Mutex& mutex);
		~ScopedLock();
		void unlock();
	private:
		Mutex& mutex;
	};

};

#endif // ORG_EEROS_CORE_SCOPEDLOCK_HPP_
