#include <eeros/hal/DummyLogicInput.hpp>

using namespace eeros::hal;

DummyLogicInput::DummyLogicInput(std::string id) : PeripheralInput<bool>(id) { }

bool DummyLogicInput::get() const {
	return false;
}
