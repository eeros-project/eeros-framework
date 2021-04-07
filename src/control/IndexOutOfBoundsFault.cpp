#include <eeros/control/IndexOutOfBoundsFault.hpp>

using namespace eeros::control;

IndexOutOfBoundsFault::IndexOutOfBoundsFault() { }

IndexOutOfBoundsFault::IndexOutOfBoundsFault(std::string m) : Fault(m) { }

IndexOutOfBoundsFault::~IndexOutOfBoundsFault() throw() { }