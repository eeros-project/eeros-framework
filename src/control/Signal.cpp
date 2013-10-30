#include <eeros/control/Signal.hpp>
#include <sstream>

using namespace eeros::control;

uint16_t Signal::signalCounter = 1;
std::list<Signal*> Signal::signalList;

Signal::Signal(sigdim_t dim) : dimension(dim) {
	majorId = signalCounter++;
	signalList.push_back(this);
}

Signal::~Signal() {
	signalList.remove(this);
}

sigmajorid_t Signal::getMajorId() const {
	return majorId;
}

sigid_t Signal::getSignalId(sigindex_t index) const {
	sigid_t id = ((sigid_t) majorId) << 16;
	id |= index;
	return id;
}

sigdim_t Signal::getDimension() const {
    return dimension;
}

std::string Signal::getLabel() const {
	return getLabel(0);
}

std::string Signal::getLabel(sigindex_t index) const {
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
	id >>= 16;
	while (i != signalList.end()) {
		if ((*i)->getMajorId() == id) {
			return (*i);
		}
		i++;
	}
	return NULL;
}
