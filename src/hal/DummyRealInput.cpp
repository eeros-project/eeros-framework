#include <eeros/hal/DummyRealInput.hpp>

using namespace eeros::hal;

DummyRealInput::DummyRealInput(std::string id, void* libHandle, double scale, double offset, double minIn, double maxIn) : ScalableInput<double>(id, libHandle, scale, offset, minIn, maxIn) { }

double DummyRealInput::get() {
	return 0;
}
