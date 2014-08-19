#include <eeros/hal/DummyRealInput.hpp>

using namespace eeros::hal;

DummyRealInput::DummyRealInput(std::string id, double scale, double offset) : ScalablePeripheralInput<double>(id, scale, offset) { }

double DummyRealInput::get() const {
	return 0;
}
