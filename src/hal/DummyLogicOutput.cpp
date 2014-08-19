#include <eeros/hal/DummyLogicOutput.hpp>

using namespace eeros::hal;

DummyLogicOutput::DummyLogicOutput(std::string id) : PeripheralOutput<bool>(id) { }

bool DummyLogicOutput::get() const {
	return false;
}

void DummyLogicOutput::set(bool value) {
	
}
