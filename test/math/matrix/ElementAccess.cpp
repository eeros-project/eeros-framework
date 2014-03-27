#include <iostream>
#include <eeros/math/Matrix.hpp>
#include "../../Utils.hpp"

#define DEFAULT_TOL 0.0001

using namespace eeros::math;

template < uint8_t M, uint8_t N, typename T >
void print(Matrix<M, N, T> &A, int indent = 1) {
	for(int m = 0; m < M; m++) {
		for(int i = 0; i < indent; i++) std::cout << '\t';
		for(int n = 0; n < N; n++) {
			if(n > 0) std::cout << '\t';
			std::cout << A(m, n);
		}
		std::cout << std::endl;
	}
}

int main(int argc, char *argv[]) {
	int error = 0, errorSum = 0;
	int testNo = 1;
	
	Matrix<3, 3, int> i123;
	Matrix<3, 3, double> d123;
	
	int i;
	double d;
	
	std::cout << "Testing element access for class Matrix" << std::endl;
	
	// ********** Part A **********
	
	std::cout << "[A] Writing integer values (-4..+4) into a 3x3 matrix, using operator (m, n)" << std::endl;
	i = -4;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			i123(m, n) = i++;
		}
	}
	
	std::cout << "    #" << testNo++ << ": Reading integer values from matrix, using operator (m, n)" << std::endl;
	error = 0;
	i = -4;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(i != i123(m, n)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << i123(m, n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
			i++;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << " Reading integer values from matrix, using operator(i)" << std::endl;
	error = 0;
	i = -4;
	for(unsigned int x = 0; x < 9; x++) {
		if(i != i123(x)) {
			std::cout << "    -> Failure: M(" << x << ") = " << i123(x) << ", but should be " << i << '!' << std::endl;
			error++;
		}
		i++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << " Reading integer values from matrix, using operator[i]" << std::endl;
	error = 0;
	i = -4;
	for(unsigned int x = 0; x < 9; x++) {
		if(i != i123[x]) {
			std::cout << "    -> Failure: M(" << x << ") = " << i123[x] << ", but should be " << i << '!' << std::endl;
			error++;
		}
		i++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Reading integer values from matrix, using get(m, n)" << std::endl;
	error = 0;
	i = -4;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(i != i123.get(m, n)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << i123.get(m, n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
			i++;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	// ********** Part B **********
	
	std::cout << "[B] Writing integer values (100..108) into a 3x3 matrix, using operator (i)" << std::endl;
	error = 0;
	i = 100;
	for(unsigned int x = 0; x < 9; x++) {
			i123(x) = i++;
	}
	
	std::cout << "    #" << testNo++ << ": Reading integer values from matrix, using operator (m, n)" << std::endl;
	error = 0;
	i = 100;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(i != i123(m, n)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << i123(m, n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
			i++;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << " Reading integer values from matrix, using operator(i)" << std::endl;
	error = 0;
	i = 100;
	for(unsigned int x = 0; x < 9; x++) {
		if(i != i123(x)) {
			std::cout << "    -> Failure: M(" << x << ") = " << i123(x) << ", but should be " << i << '!' << std::endl;
			error++;
		}
		i++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << " Reading integer values from matrix, using operator[i]" << std::endl;
	error = 0;
	i = 100;
	for(unsigned int x = 0; x < 9; x++) {
		if(i != i123[x]) {
			std::cout << "    -> Failure: M(" << x << ") = " << i123[x] << ", but should be " << i << '!' << std::endl;
			error++;
		}
		i++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Reading integer values from matrix, using get(m, n)" << std::endl;
	error = 0;
	i = 100;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(i != i123.get(m, n)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << i123.get(m, n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
			i++;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	// ********** Part C **********
	
	std::cout << "[C] Writing integer values (-10..-18) into a 3x3 matrix, using operator [i]" << std::endl;
	error = 0;
	i = -10;
	for(unsigned int x = 0; x < 9; x++) {
			i123[x] = i++;
	}
	
	std::cout << "    #" << testNo++ << ": Reading integer values from matrix, using operator (m, n)" << std::endl;
	error = 0;
	i = -10;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(i != i123(m, n)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << i123(m, n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
			i++;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << " Reading integer values from matrix, using operator(i)" << std::endl;
	error = 0;
	i = -10;
	for(unsigned int x = 0; x < 9; x++) {
		if(i != i123(x)) {
			std::cout << "    -> Failure: M(" << x << ") = " << i123(x) << ", but should be " << i << '!' << std::endl;
			error++;
		}
		i++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << " Reading integer values from matrix, using operator[i]" << std::endl;
	error = 0;
	i = -10;
	for(unsigned int x = 0; x < 9; x++) {
		if(i != i123[x]) {
			std::cout << "    -> Failure: M(" << x << ") = " << i123[x] << ", but should be " << i << '!' << std::endl;
			error++;
		}
		i++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Reading integer values from matrix, using get(m, n)" << std::endl;
	error = 0;
	i = -10;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(i != i123.get(m, n)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << i123.get(m, n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
			i++;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	// ********** Part D **********
	
	std::cout << "[D] Writing integer values (1000..1008) into a 3x3 matrix, using operator set(m, n)" << std::endl;
	error = 0;
	i = 1000;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			i123.set(m, n, i++);
		}
	}
	
	std::cout << "    #" << testNo++ << ": Reading integer values from matrix, using operator (m, n)" << std::endl;
	error = 0;
	i = 1000;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(i != i123(m, n)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << i123(m, n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
			i++;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << " Reading integer values from matrix, using operator(i)" << std::endl;
	error = 0;
	i = 1000;
	for(unsigned int x = 0; x < 9; x++) {
		if(i != i123(x)) {
			std::cout << "    -> Failure: M(" << x << ") = " << i123(x) << ", but should be " << i << '!' << std::endl;
			error++;
		}
		i++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << " Reading integer values from matrix, using operator[i]" << std::endl;
	error = 0;
	i = 1000;
	for(unsigned int x = 0; x < 9; x++) {
		if(i != i123[x]) {
			std::cout << "    -> Failure: M(" << x << ") = " << i123[x] << ", but should be " << i << '!' << std::endl;
			error++;
		}
		i++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Reading integer values from matrix, using get(m, n)" << std::endl;
	error = 0;
	i = 1000;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(i != i123.get(m, n)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << i123.get(m, n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
			i++;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	// ********** Part E **********
	
	std::cout << "[E] Writing double values (-4.0..+4.0) into a 3x3 matrix, using operator (m, n)" << std::endl;
	error = 0;
	d = -4;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			d123(m, n) = d;
			d += 1.0;
		}
	}
	errorSum += error;
	std::cout << "    Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Reading double values from matrix, using operator (m, n)" << std::endl;
	error = 0;
	d = -4;
	for(unsigned int n = 0; n < 3; n++) {
		for(unsigned int m = 0; m < 3; m++) {
			if(!Utils::compareApprox(d, d123(m, n), DEFAULT_TOL)) {
				std::cout << "    -> Failure: M(" << m << ',' << n << ") = " << d123(m, n) << ", but should be " << d << '!' << std::endl;
				error++;
			}
			d += 1.0;
		}
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	
	if (errorSum == 0)
		std::cout << "Matrix element access test succeeded" << std::endl;
	else
		std::cout << "Matrix element access test failed with " << errorSum << " error(s)" << std::endl;

	return errorSum;
}
