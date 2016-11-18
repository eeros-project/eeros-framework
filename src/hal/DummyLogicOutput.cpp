#include <eeros/hal/DummyLogicOutput.hpp>

using namespace eeros::hal;

DummyLogicOutput::DummyLogicOutput(std::string id) : PeripheralOutput<bool>(id) { }

bool DummyLogicOutput::get() {
	return value;
}

void DummyLogicOutput::set(bool val) {
	value = val;
}
