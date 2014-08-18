#include <eeros/core/Thread.hpp>
#include <sstream>

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

std::string Thread::getId() const {
	std::ostringstream s;
	s << t.get_id();
	return s.str();
}

void Thread::join() {
	if(t.joinable()) t.join();
}
