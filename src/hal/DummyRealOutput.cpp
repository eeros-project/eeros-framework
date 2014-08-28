#include <eeros/hal/DummyRealOutput.hpp>

using namespace eeros::hal;

DummyRealOutput::DummyRealOutput(std::string id, double scale, double offset) : ScalablePeripheralOutput<double>(id, scale, offset) { }

double DummyRealOutput::get() {
	return 0;
}

void DummyRealOutput::set(double value) {
	
}
