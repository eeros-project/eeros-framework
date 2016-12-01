#include <eeros/hal/DummyRealInput.hpp>

using namespace eeros::hal;

DummyRealInput::DummyRealInput(std::string id, double scale, double offset) : ScalableInput<double>(id, scale, offset) { }

double DummyRealInput::get() {
	return 0;
}
