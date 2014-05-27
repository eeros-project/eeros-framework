#include <eeros/math/Frame.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros;
using namespace eeros::math;

Frame::Frame(const CoordinateSystem& a, const CoordinateSystem& b) : a(a), b(b) {
	T.eye();
}

Frame::Frame(const CoordinateSystem& a, const CoordinateSystem& b, eeros::math::Matrix<4, 4, double> T) : a(a), b(b), T(T) {
	// nothing to do
}

Frame::Frame(const CoordinateSystem& a, const CoordinateSystem& b, eeros::math::Matrix<3, 3, double> R, eeros::math::Matrix<3, 1, double> r) : a(a), b(b) {
	set(R, r);
}

void Frame::set(eeros::math::Matrix<4, 4, double> T) {
	this->T = T;
}

void Frame::set(eeros::math::Matrix<3, 3, double> R, eeros::math::Matrix<3, 1, double> r) {
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

Frame Frame::operator*(const Frame& right) const {
	if(b != right.a) {
		throw EEROSException("Frame coordinate systems incompatible!");
	}
	Frame result(a, right.b, T * right.T);
	return result;
}
