#include <eeros/control/Signal.hpp>
#include <sstream>

uint16_t Signal::signalCounter = 1;
std::list<Signal*> Signal::signalList;

Signal::Signal(sigdim_t dim) : dimension(dim) {
	majorId = signalCounter++;
	signalList.push_back(this);
}

Signal::~Signal() {
	signalList.remove(this);
}

sigid_t Signal::getSignalId() const {
	return ((sigid_t) majorId) << 16;
}

sigid_t Signal::getSignalId(sigindex_t index) const {
	return (((sigid_t) majorId) << 16) || index;
}

sigdim_t Signal::getDimension() const {
    return dimension;
}

std::string Signal::getLabel() const {
	return getLabel(0);
}

std::string Signal::getLabel(int index) const {
	std::stringstream label;
	label << '#' << majorId << '/' << index;
	return label.str();
}

sigtype_t Signal::getType() const {
	return kSignalTypeUnknown;
}

std::list<Signal*>* Signal::getSignalList() {
	return &signalList;
}

Signal* Signal::getSignalById(sigid_t id) {
	std::list<Signal*>::iterator i = signalList.begin();
	while ((*i)->getSignalId() != id && i != signalList.end()) {
		i++;
	}
	return (*i);
}
