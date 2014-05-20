#include <eeros/core/Thread.hpp>

using namespace eeros;

Thread::Thread() : t([&]() { this->run(); }) {
	// nothing to do
}

Thread::Thread(std::function<void ()> t) : t(t) {
	// nothing to do
}

Thread::~Thread() {
	join();
}

std::thread::id Thread::getId() const {
	return t.get_id();
}

void Thread::join() {
	if(t.joinable()) t.join();
}
