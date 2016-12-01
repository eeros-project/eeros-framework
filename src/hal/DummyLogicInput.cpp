#include <eeros/hal/DummyLogicInput.hpp>

using namespace eeros::hal;

DummyLogicInput::DummyLogicInput(std::string id) : Input<bool>(id) { }

bool DummyLogicInput::get() {
	return false;
}
