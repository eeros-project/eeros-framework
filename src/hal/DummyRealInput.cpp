#include <eeros/hal/DummyRealInput.hpp>

using namespace eeros::hal;

DummyRealInput::DummyRealInput(std::string id, double scale, double offset, double minIn, double maxIn) : ScalableInput<double>(id, scale, offset, minIn, maxIn) { }

double DummyRealInput::get() {
	return 0;
}
