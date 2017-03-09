#include <eeros/control/NotConnectedFault.hpp>

using namespace eeros::control;

NotConnectedFault::NotConnectedFault() { }

NotConnectedFault::NotConnectedFault(std::string m) : Fault(m) { }

NotConnectedFault::~NotConnectedFault() throw() { }

