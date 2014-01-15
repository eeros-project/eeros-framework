#ifndef ORG_EEROS_CORE_MUTEX_HPP_
#define ORG_EEROS_CORE_MUTEX_HPP_

#include <pthread.h>

namespace eeros {

	class Mutex {
	public:
		Mutex();
		~Mutex();
		void lock();
		void unlock();
	private:
		pthread_mutex_t mutex;
	};

};

#endif // ORG_EEROS_CORE_MUTEX_HPP_
