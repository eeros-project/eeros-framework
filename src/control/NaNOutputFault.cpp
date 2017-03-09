#include <eeros/control/NaNOutputFault.hpp>

using namespace eeros::control;

NaNOutputFault::NaNOutputFault() { }

NaNOutputFault::NaNOutputFault(std::string m) : Fault(m) { }

NaNOutputFault::~NaNOutputFault() throw() { }

