#include <eeros/core/Lock.hpp>
using namespace eeros;

Lock::Lock(Mutex& mutex) : mutex(mutex) {

}

Lock::~Lock() {
	mutex.unlock();
}

void Lock::lock() {
	mutex.lock();
}

void Lock::unlock() {
	mutex.unlock();
}
