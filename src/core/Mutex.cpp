#include <eeros/core/Mutex.hpp>
using namespace eeros;

Mutex::Mutex() {
	pthread_mutex_init(&mutex, 0);
}

Mutex::~Mutex() {
	pthread_mutex_destroy(&mutex);
}

void Mutex::lock() {
	pthread_mutex_lock(&mutex);
}

void Mutex::unlock() {
	pthread_mutex_unlock(&mutex);
}
