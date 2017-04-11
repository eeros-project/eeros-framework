#include <eeros/hal/DummyRealOutput.hpp>

using namespace eeros::hal;

DummyRealOutput::DummyRealOutput(std::string id, void* libHandle, double scale, double offset, double minOut, double maxOut) : ScalableOutput<double>(id, libHandle, scale, offset, minOut, maxOut) { }

double DummyRealOutput::get() {
	return 0;
}

void DummyRealOutput::set(double value) {
	
}
