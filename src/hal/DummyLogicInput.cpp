#include <eeros/hal/DummyLogicInput.hpp>

using namespace eeros::hal;

DummyLogicInput::DummyLogicInput(std::string id, void* libHandle) : Input<bool>(id, libHandle) { }

bool DummyLogicInput::get() {
	return false;
}
