#include <eeros/math/Frame.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros;
using namespace eeros::math;

std::list<Frame*> Frame::list;

Frame::Frame(const CoordinateSystem& a, const CoordinateSystem& b) : a(a), b(b) {
	T.eye();
	if(getFrame(a, b) != nullptr) {
		std::stringstream msg;
		msg << "Frame with a = '" << a << "' and b = '" << b << "' exists already!";
		throw EEROSException(msg.str());
	}
	list.push_back(this);
}

Frame::Frame(const CoordinateSystem& a, const CoordinateSystem& b, const eeros::math::Matrix<4, 4, double>& T) : a(a), b(b), T(T) {
	if(getFrame(a, b) != nullptr) {
		std::stringstream msg;
		msg << "Frame with a = '" << a << "' and b = '" << b << "' exists already!";
		throw EEROSException(msg.str());
	}
	list.push_back(this);
}

Frame::Frame(const CoordinateSystem& a, const CoordinateSystem& b, const eeros::math::Matrix<3, 3, double>& R, const eeros::math::Matrix<3, 1, double>& r) : a(a), b(b) {
	set(R, r);
	if(getFrame(a, b) != nullptr) {
		std::stringstream msg;
		msg << "Frame with a = '" << a << "' and b = '" << b << "' exists already!";
		throw EEROSException(msg.str());
	}
	list.push_back(this);
}

Frame::~Frame() {
	list.remove(this);
}

void Frame::set(const eeros::math::Matrix<4, 4, double>& T) {
	this->T = T;
}

void Frame::set(const eeros::math::Matrix<3, 3, double>& R, const eeros::math::Matrix<3, 1, double>& r) {
	for(int n = 0; n < 3; n++) {
		for(int m = 0; m < 3; m++) {
			T(m, n) = R(m,n);
		}
	}
	for(int m = 0; m < 3; m++) {
		T(m, 3) = r(m);
	}
	T(3, 0) = 0; T(3, 1) = 0; T(3, 2) = 0;
	T(3, 3) = 1;
}

eeros::math::Matrix<4, 4, double> Frame::get() const {
	return T;
}

const CoordinateSystem& Frame::getFromCoordinateSystem() const {
	return a;
}

const CoordinateSystem& Frame::getToCoordinateSystem() const {
	return b;
}

Frame Frame::operator*(const Frame& right) const {
	if(b != right.a) {
		throw EEROSException("Frame coordinate systems incompatible!");
	}
	Frame result(a, right.b, T * right.T);
	return result;
}

Frame* Frame::getFrame(const CoordinateSystem& a, const CoordinateSystem& b) {
	for(auto f : list) {
		if(f->getFromCoordinateSystem() == a && f->getToCoordinateSystem() == b) return f;
	}
	return nullptr;
}

uint32_t Frame::getNofFrames() {
	return list.size();
}
