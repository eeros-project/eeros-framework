#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>

double RealSignalInput::getValue() {
	return getValue(0);
}

double RealSignalInput::getValue(int index) {
	if(connectedOutput) {
		return dynamic_cast<RealSignalOutput*>(connectedOutput)->getValue(index);
	}
	return 0;
}

sigdim_t RealSignalInput::getDimension() {
	if(connectedOutput) {
		return dynamic_cast<RealSignalOutput*>(connectedOutput)->getDimension();
	}
	return 0;
}

uint64_t RealSignalInput::getTimestamp() {
	return getTimestamp(0);
}

uint64_t RealSignalInput::getTimestamp(int index) {
	if(connectedOutput) {
		return dynamic_cast<RealSignalOutput*>(connectedOutput)->getTimestamp(index);
	}
	return 0;
}

std::string RealSignalInput::getName() {
	return getName(0);
}

std::string RealSignalInput::getName(int index) {
	if(connectedOutput) {
		return dynamic_cast<RealSignalOutput*>(connectedOutput)->getName(index);
	}
    std::string empty;
	return empty;
}

std::string RealSignalInput::getUnit() {
	return getUnit(0);
}

std::string RealSignalInput::getUnit(int index) {
	if(connectedOutput) {
		return dynamic_cast<RealSignalOutput*>(connectedOutput)->getUnit(index);
	}
    std::string empty;
	return empty;
}

std::string RealSignalInput::getCoordinateSystem() {
	return getCoordinateSystem(0);
}

std::string RealSignalInput::getCoordinateSystem(int index) {
	if(connectedOutput) {
		return dynamic_cast<RealSignalOutput*>(connectedOutput)->getCoordinateSystem(index);
	}
    std::string empty;
	return empty;
}
