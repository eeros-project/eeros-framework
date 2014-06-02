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
	
	std::cout << "Testing matrix operations, part 2" << std::endl;
	
	// ********** Part A **********
	
	Matrix<3, 3, int> matA1_test;
	Matrix<3, 3, int> matA1_res;
	Matrix<3, 3, int> matA1_ref;
	Matrix<2, 3, int> matA2_test;
	Matrix<3, 2, int> matA2_res;
	Matrix<3, 2, int> matA2_ref;
	Matrix<3, 1, int> matA3_test;
	Matrix<1, 3, int> matA3_res;
	Matrix<1, 3, int> matA3_ref;
	
	std::cout << "[A] transpose()" << std::endl;
	
	std::cout << "    #" << testNo++ << ": 3x3 matrix" << std::endl;
	error = 0;
	
	matA1_test << 1, 2, 3,
	              4, 5, 6,
	              7, 8, 9;
	matA1_ref  << 1, 4, 7,
	              2, 5, 8,
	              3, 6, 9;
	matA1_res = matA1_test.transpose();
	
	if(matA1_res != matA1_ref) {
		std::cout << "    -> Failure: Transpose of 3x3 matrix failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": 2x3 matrix" << std::endl;
	error = 0;
	
	matA2_test << 1, 2, 3,
	              4, 5, 6;
	matA2_ref  << 1, 4,
	              2, 5,
	              3, 6;
	matA2_res = matA2_test.transpose();
	
	if(matA2_res != matA2_ref) {
		std::cout << "    -> Failure: Transpose of 2x3 matrix failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": 3x1 matrix" << std::endl;
	error = 0;
	
	matA3_test << 1,
	              2,
	              3;
	matA3_ref  << 1, 2, 3;
	matA3_res = matA3_test.transpose();
	
	if(matA3_res != matA3_ref) {
		std::cout << "    -> Failure: Transpose of 3x1 matrix failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	// ********** END **********
	
	if(errorSum == 0) {
		std::cout << "Matrix operations test 2 succeeded" << std::endl;
	}
	else {
		std::cout << "Matrix operations test 2 failed with " << errorSum << " error(s)" << std::endl;
	}
	
	return errorSum;
}
