#include <eeros/control/Signal.hpp>
#include <sstream>

uint32_t Signal::signalCounter = 0;
std::list<Signal*> Signal::signalList;

Signal::Signal(sigdim_t dim) : length(dim) {
	id = signalCounter++;
	signalList.push_back(this);
}

Signal::~Signal() {
	signalList.remove(this);
}

std::list<Signal*>* Signal::getSignalList() {
	return &signalList;
}

uint32_t Signal::getSignalId() {
	return id;
}

uint32_t Signal::getLength()
{
    return length;
}

std::string Signal::getLabel() {
	return getLabel(0);
}

std::string Signal::getLabel(int index) {
	std::stringstream label;
	label << '#' << id << '/' << index;
	return label.str();
}