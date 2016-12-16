#include <eeros/hal/DummyLogicOutput.hpp>

using namespace eeros::hal;

DummyLogicOutput::DummyLogicOutput(std::string id, void* libHandle) : Output<bool>(id, libHandle) { }

bool DummyLogicOutput::get() {
	return false;
}

void DummyLogicOutput::set(bool value) {
	
}
