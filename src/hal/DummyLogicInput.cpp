#include <eeros/hal/DummyLogicInput.hpp>

using namespace eeros::hal;

DummyLogicInput::DummyLogicInput(std::string id) : PeripheralInput<bool>(id) { }

bool DummyLogicInput::get() {
	return value;
}

void DummyLogicInput::set(bool val) {
	value = val;
}
