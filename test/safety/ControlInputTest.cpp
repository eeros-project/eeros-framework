#include <iostream>

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/safety/ControlInput.hpp>

#define ERROR(msg_) (std::cout << "ERROR: \"" << msg_ << "\" [line " << __LINE__ << "] " << __FILE__ << std::endl, errors++)
#define EXPECTING(expected_, unexpected_) (ERROR("expecting " << expected_ << ", got " << unexpected_))


int test_double() {
	int errors = 0;
	
	eeros::control::Output<double> out1;
	
	eeros::control::Input<double> in1;
	in1.connect(out1);
	
	eeros::safety::ControlInput<double> cin1("in1", in1);
	
	double x;
	double y;
	
	x = 0.0;
	out1.getSignal().setValue(x);
	y = cin1.get();
	if (x != y) EXPECTING(x, y);
	
	x = 3.5;
	out1.getSignal().setValue(x);
	y = cin1.get();
	if (x != y) EXPECTING(x, y);
	
	x = -587446.06548;
	out1.getSignal().setValue(x);
	y = cin1.get();
	if (x != y) EXPECTING(x, y);
	
	return errors;
}

int test_vector() {
	int errors = 0;
	
	using T = eeros::math::Vector<3>;
	
	eeros::math::Matrix<3,3> A = {
		1.1, 2.2, 3.3,
		4.4, 5.5, 6.6,
		7.7, 8.8, 9.9
	};
	
	eeros::control::Output<T> out1;
	
	eeros::control::Input<T> in1;
	in1.connect(out1);
	
	eeros::safety::ControlInput<T> cin1("in1", in1);
	
	T x;
	T y;
	
	x = 0.0;
	out1.getSignal().setValue(x);
	y = cin1.get();
	if (x != y) EXPECTING(x, y);
	
	x[0] = 0.1;
	x[1] = 0.2;
	x[1] = 0.3;
	out1.getSignal().setValue(x);
	y = cin1.get();
	if (x != y) EXPECTING(x, y);
	
	x = A*x;
	out1.getSignal().setValue(x);
	y = cin1.get();
	if (x != y) EXPECTING(x, y);
	
	x = A*x;
	out1.getSignal().setValue(x);
	y = cin1.get();
	if (x != y) EXPECTING(x, y);
	
	return errors;
}


int main() {
	int errors = 0;
	
	errors += test_double();
	errors += test_vector();
	
	std::cout << "test finished with " << errors << " error(s)" << std::endl;
	return errors;
}
