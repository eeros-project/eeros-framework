#include <eeros/core/ScopedLock.hpp>
using namespace eeros;

ScopedLock::ScopedLock(Mutex& mutex) : mutex(mutex) {
	mutex.lock();
}

ScopedLock::~ScopedLock() {
	mutex.unlock();
}

void ScopedLock::unlock() {
	mutex.unlock();
}
