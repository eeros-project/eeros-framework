#include <eeros/math/Frame.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros;
using namespace eeros::math;

std::map<std::string, CoordinateSystem*> CoordinateSystem::list;

CoordinateSystem::CoordinateSystem(const CoordinateSystem&) { }

CoordinateSystem& CoordinateSystem::operator=(const CoordinateSystem&) { }

CoordinateSystem::CoordinateSystem(std::string id) : id(id) {
	if(!CoordinateSystem::list.insert( {id, this} ).second) {
		std::stringstream msg;
		msg << "Coordinate system with id '" << id << "' exists already, pleace choose a unique name!";
		throw EEROSException(msg.str());
	}
}

CoordinateSystem::~CoordinateSystem() {
	CoordinateSystem::list.erase(id);
}

bool CoordinateSystem::operator==(const CoordinateSystem& right) const {
	return this == &right;
}

bool CoordinateSystem::operator!=(const CoordinateSystem& right) const {
	return this != &right;
}

CoordinateSystem* CoordinateSystem::getCoordinateSystem(std::string id) {
	return CoordinateSystem::list[id];
}
