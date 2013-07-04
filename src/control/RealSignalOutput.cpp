#include <eeros/control/RealSignalOutput.hpp>
#include <sstream>

RealSignalOutput::RealSignalOutput(sigdim_t dim) : Signal(dim) {
	this->length = dim;
	this->dat = new anDatum[dim];
	for(sigdim_t i = 0; i < dim; i++) {
		this->dat[i].value = 0;
		this->dat[i].timestamp = 0;
		this->dat[i].name = "unnamed";
		this->dat[i].unit = "";
		this->dat[i].coordinateSystem = "";
	}
}

// RealSignalOutput::RealSignalOutput(std::string signalName, std::string unit, std::string coordinateSystem) {
// 	this->length = 1;
// 	this->dat = new anDatum[1];
// 	this->dat[0].value = 0;
// 	this->dat[0].timestamp = 0;
// 	this->dat[0].name = signalName;
// 	this->dat[0].unit = unit;  // TODO check if given
// 	this->dat[0].coordinateSystem = coordinateSystem;  // TODO check if given
// }
// 
// RealSignalOutput::RealSignalOutput(std::string signalName[], std::string unit[], std::string coordinateSystem[], int length) {
//     this->length = length;
// 	this->dat = new anDatum[length];
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
	for (int i = 0; i < length; i++) {
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

std::string RealSignalOutput::getLabel(int index) const {
	std::stringstream label;
	label << '#' << id << '/' << index << ": " << getName(index) << " [" << getUnit(index) << ']';
	return label.str();
}

double RealSignalOutput::getValue() const {
	return getValue(0);
}

double RealSignalOutput::getValue(int index) const {
	if(index < length) return dat[index].value;
	return 0;
}

uint64_t RealSignalOutput::getTimestamp() const {
	return getTimestamp(0);
}

uint64_t RealSignalOutput::getTimestamp(int index) const {
	if(index < length) return dat[index].timestamp;
	return 0;
}

std::string RealSignalOutput::getName() const {
	return getName(0);
}

std::string RealSignalOutput::getName(int index) const {
	if(index < length) return dat[index].name;
    std::string empty;
	return empty;
}

std::string RealSignalOutput::getUnit() const {
	return getUnit(0);
}

std::string RealSignalOutput::getUnit(int index) const {
	if(index < length) return dat[index].unit;
    std::string empty;
	return empty;
}

std::string RealSignalOutput::getCoordinateSystem() const {
	return getCoordinateSystem(0);
}

std::string RealSignalOutput::getCoordinateSystem(int index) const {
	if(index < length) return dat[index].coordinateSystem;
    std::string empty;
	return empty;
}

void RealSignalOutput::setValue(double newValue) {
	setValue(newValue, 0);
}

void RealSignalOutput::setValue(double newValue, int index) {
	dat[index].value = newValue;
}

void RealSignalOutput::setValue(double newValue[]) { // TODO check if array lenght > length
	for(int i = 0; i < length; i++)
	{
		dat[i].value = newValue[i];
	}
}

void RealSignalOutput::setTimeStamp(uint64_t timestamp) {
	setTimeStamp(timestamp, 0);
}

void RealSignalOutput::setTimeStamp(uint64_t timestamp, int index) {
	if(index < length) dat[index].timestamp = timestamp;
}

void RealSignalOutput::setName(std::string signalName) {
	setName(signalName, 0);
}

void RealSignalOutput::setName(std::string signalName, int index) {
	if(index < length) dat[index].name = signalName;
}

void RealSignalOutput::setUnit(std::string unit) {
	setUnit(unit, 0);
}

void RealSignalOutput::setUnit(std::string unit, int index) {
	if(index < length) dat[index].unit = unit;
}

void RealSignalOutput::setCoordinateSystem(std::string coordinateSystem) {
	setCoordinateSystem(coordinateSystem, 0);
}

void RealSignalOutput::setCoordinateSystem(std::string coordinateSystem, int index) {
	if(index < length) dat[index].coordinateSystem = coordinateSystem;
}
