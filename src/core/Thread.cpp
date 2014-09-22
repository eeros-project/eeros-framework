#include <eeros/core/Thread.hpp>
#include <sstream>

using namespace eeros;

Thread::Thread() : t([&]() {
	std::string id = getId();
	log.trace() << "Thread '" << id << "' started.";
	this->run();
	log.trace() << "Thread '" << id << "' finished.";
}) { }

Thread::Thread(std::function<void ()> t) : t(t) { }

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
