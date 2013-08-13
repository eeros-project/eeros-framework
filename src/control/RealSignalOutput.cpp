#include <eeros/control/RealSignalOutput.hpp>
#include <sstream>

RealSignalOutput::RealSignalOutput(sigdim_t dim) : Signal(dim) {
	this->dimension = dim;
	this->dat = new realSignalDatum[dim];
	for(sigdim_t i = 0; i < dim; i++) {
		this->dat[i].value = 0;
		this->dat[i].timestamp = 0;
		this->dat[i].name = "unnamed";
		this->dat[i].unit = "";
		this->dat[i].coordinateSystem = "";
		this->dat[i].sendingDirection = "ServerToClient";
	}
}

// RealSignalOutput::RealSignalOutput(std::string signalName, std::string unit, std::string coordinateSystem) {
// 	this->length = 1;
// 	this->dat = new realSignalDatum[1];
// 	this->dat[0].value = 0;
// 	this->dat[0].timestamp = 0;
// 	this->dat[0].name = signalName;
// 	this->dat[0].unit = unit;  // TODO check if given
// 	this->dat[0].coordinateSystem = coordinateSystem;  // TODO check if given
// }
// 
// RealSignalOutput::RealSignalOutput(std::string signalName[], std::string unit[], std::string coordinateSystem[], int length) {
//     this->length = length;
// 	this->dat = new realSignalDatum[length];
// 	for (int i = 0; i < length; i++) {
// 		dat[i].value = 0;
// 		dat[i].timestamp = 0;
// //		if (signalName[i] != NULL)
// 			dat[i].name = signalName[i];
// //		else
// //			dat[i].name = "";
// //		if (unit[i] != NULL)
// 			dat[i].unit = unit[i];
// //		else
// //			dat[i].unit = "";
// //		if (coordinateSystem[i] != NULL)
// 			dat[i].coordinateSystem = coordinateSystem[i];
// //		else
// //			dat[i].coordinateSystem = "";
// 	}
// }

RealSignalOutput::~RealSignalOutput() {
	for (int i = 0; i < dimension; i++) {
		//delete &(dat[i]);
	}
	signalList.remove(this);
}

sigtype_t RealSignalOutput::getType() const {
	return kSignalTypeReal;
}

std::string RealSignalOutput::getLabel() const {
	return getLabel(0);
}

std::string RealSignalOutput::getLabel(sigindex_t index) const {
	std::stringstream label;
	label << '#' << majorId << '/' << index << ": " << getName(index);
	if (getCoordinateSystem(index) != "") {
		label << '_' << getCoordinateSystem(index);
	}
	if (getUnit(index) != "") {
		label << " [" << getUnit(index) << ']';
	}
	return label.str();
}

double RealSignalOutput::getValue() const {
	return getValue(0);
}

double RealSignalOutput::getValue(sigindex_t index) const {
	if(index < dimension) return dat[index].value;
	return 0;
}

uint64_t RealSignalOutput::getTimestamp() const {
	return getTimestamp(0);
}

uint64_t RealSignalOutput::getTimestamp(sigindex_t index) const {
	if(index < dimension) return dat[index].timestamp;
	return 0;
}

std::string RealSignalOutput::getName() const {
	return getName(0);
}

std::string RealSignalOutput::getName(sigindex_t index) const {
	if(index < dimension) return dat[index].name;
    std::string empty;
	return empty;
}

std::string RealSignalOutput::getUnit() const {
	return getUnit(0);
}

std::string RealSignalOutput::getUnit(sigindex_t index) const {
	if(index < dimension) return dat[index].unit;
    std::string empty;
	return empty;
}

std::string RealSignalOutput::getCoordinateSystem() const {
	return getCoordinateSystem(0);
}

std::string RealSignalOutput::getCoordinateSystem(sigindex_t index) const {
	if(index < dimension) return dat[index].coordinateSystem;
    std::string empty;
	return empty;
}

std::string RealSignalOutput::getSendingDirection() const {
	return getSendingDirection(0);
}

std::string RealSignalOutput::getSendingDirection(sigindex_t index) const {
	if(index < dimension) return dat[index].sendingDirection;
    std::string empty;
	return empty;
}

void RealSignalOutput::setValue(double newValue) {
	setValue(newValue, 0);
}

void RealSignalOutput::setValue(double newValue, sigindex_t index) {
	dat[index].value = newValue;
}

void RealSignalOutput::setValue(double newValue[]) { // TODO check if array lenght > length
	for(int i = 0; i < dimension; i++) {
		dat[i].value = newValue[i];
	}
}

void RealSignalOutput::setTimeStamp(uint64_t timestamp) {
	for(int i = 0; i < dimension; i++) {
		dat[i].timestamp = timestamp;
	}
}

void RealSignalOutput::setTimeStamp(uint64_t timestamp, sigindex_t index) {
	if(index < dimension) dat[index].timestamp = timestamp;
}

void RealSignalOutput::setName(std::string signalName) {
	for(int i = 0; i < dimension; i++) {
		dat[i].name = signalName;
	}
}

void RealSignalOutput::setName(std::string signalName, sigindex_t index) {
	if(index < dimension) dat[index].name = signalName;
}

void RealSignalOutput::setUnit(std::string unit) {
	for(int i = 0; i < dimension; i++) {
		dat[i].unit = unit;
	}
}

void RealSignalOutput::setUnit(std::string unit, sigindex_t index) {
	if(index < dimension) dat[index].unit = unit;
}

void RealSignalOutput::setCoordinateSystem(std::string coordinateSystem) {
	for(int i = 0; i < dimension; i++) {
		dat[i].coordinateSystem = coordinateSystem;
	}
}

void RealSignalOutput::setCoordinateSystem(std::string coordinateSystem, sigindex_t index) {
	if(index < dimension) dat[index].coordinateSystem = coordinateSystem;
}

void RealSignalOutput::setSendingDirection(std::string sendingDirection) {
	for(int i = 0; i < dimension; i++) {
		dat[i].sendingDirection = sendingDirection;
	}
}

void RealSignalOutput::setSendingDirection(std::string sendingDirection, sigindex_t index) {
	if(index < dimension) dat[index].sendingDirection = sendingDirection;
}
