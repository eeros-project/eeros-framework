#include <eeros/math/Frame.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros;
using namespace eeros::math;

CoordinateSystem::CoordinateSystem(std::string id) : id(id) {
	// nothing to do
}

CoordinateSystem::CoordinateSystem(const CoordinateSystem&) {
	// nothing to do
}

CoordinateSystem& CoordinateSystem::operator=(const CoordinateSystem&) {
	// nothing to do
}

bool CoordinateSystem::operator==(const CoordinateSystem& right) const {
	return this == &right;
}

bool CoordinateSystem::operator!=(const CoordinateSystem& right) const {
	return this != &right;
}
