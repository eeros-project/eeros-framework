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
	
	Matrix<3, 1, int> matA_3x1;
	Matrix<3, 3, int> matA_3x3;
	Matrix<1, 2, int> matA_1x2;
	
	Matrix<3, 1, int> matA_res1;
	Matrix<3, 1, int> matA_ref1;
	Matrix<3, 3, int> matA_res2;
	Matrix<3, 3, int> matA_ref2;
	Matrix<1, 2, int> matA_res3;
	Matrix<1, 2, int> matA_ref3;
	
	std::cout << "[A] addition (+)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Sum of two 3x1 vectors" << std::endl;
	error = 0;
	
	matA_3x1 << 1,
	            2,
	            3;
	matA_ref1 << 2,
	             4,
	             6;
	matA_res1 = matA_3x1 + matA_3x1;
	
	if(matA_res1 != matA_ref1) {
		std::cout << "    -> Failure: Addtion failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	
	std::cout << "    #" << testNo++ << ": Sum of two 3x3 matrices" << std::endl;
	error = 0;
	
	matA_3x3 << 1, 2, 3,
	            4, 5, 6,
	            7, 8, 9;
	matA_ref2 << 2,  4,  6,
	             8, 10, 12,
	            14, 16, 18;
	matA_res2 = matA_3x3 + matA_3x3;
	
	if(matA_res2 != matA_ref2) {
		std::cout << "    -> Failure: Addtion failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	
	std::cout << "    #" << testNo++ << ": Sum of two 1x2 matrices" << std::endl;
	error = 0;
	
	matA_1x2 << 1, 2;
	matA_ref3 << 2, 4;
	matA_res3 = matA_1x2 + matA_1x2;
	
	if(matA_res3 != matA_ref3) {
		std::cout << "    -> Failure: Addtion failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	
	// ********** Part B **********
	
	Matrix<3, 1, int> matB_3x1a;
	Matrix<3, 1, int> matB_3x1b;
	
	Matrix<3, 1, int> matB_res1;
	Matrix<3, 1, int> matB_ref1;
	
	std::cout << "[B] subtraction (-)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Difference of two 3x1 vectors" << std::endl;
	error = 0;
	
	matB_3x1a << 9,
	             8,
	             7;
	matB_3x1a << 1,
	             2,
	             3;
	matB_ref1 << 8,
	             6,
	             4;
	matB_res1 = matB_3x1a - matB_3x1b;
	
	if(matA_res1 != matA_ref1) {
		std::cout << "    -> Failure: Subraction failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	
	// ********** Part C **********
	
	Matrix<3, 1, int> matC_3x1;
	Matrix<3, 3, int> matC_3x3;
	Matrix<2, 2, int> matC_2x2;
	Matrix<3, 2, int> matC_3x2;
	Matrix<4, 3, int> matC_4x3;

	Matrix<3, 1, int> matC_res1;
	Matrix<3, 1, int> matC_ref1;
	Matrix<2, 2, int> matC_res2;
	Matrix<2, 2, int> matC_ref2;
	Matrix<4, 2, int> matC_res3;
	Matrix<4, 2, int> matC_ref3;
	
	std::cout << "[C] multiplication (*)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": 3x3 matrix multiplied with a 3x1 matrix" << std::endl;
	error = 0;
	
	matC_3x1 << 1,
	            2,
	            3;
	matC_3x3 << 1, 4, 7,
	            2, 5, 8,
	            3, 6, 9;
	matC_ref1 << 30,
	             36,
	             42;
	matC_res1 = matC_3x3 * matC_3x1;
	
	if(matC_res1 != matC_ref1) {
		std::cout << "    -> Failure: Multiplication failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	
	std::cout << "    #" << testNo++ << ": 2x2 matrix multiplied with a 2x2 matrix" << std::endl;
	error = 0;
	
	matC_2x2 << 1, 2,
	            3, 4;
	matC_ref2 << 7, 10,
	             15, 22;
	matC_res2 = matC_2x2 * matC_2x2;
	
	if(matC_res2 != matC_ref2) {
		std::cout << "    -> Failure: Multiplication failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	
	std::cout << "    #" << testNo++ << ": 4x3 matrix multiplied with a 3x2 matrix" << std::endl;
	error = 0;
	
	matC_4x3 << 1,  2,  3,
	            4,  5,  6,
	            7,  8,  9,
	           10, 11, 12;
	matC_3x2 << 1, 2,
	            3, 4,
	            5, 6;
	matC_ref3 << 22,  28,
	             49,  64,
	             76, 100,
	            103, 136;
	matC_res3 = matC_4x3 * matC_3x2;
	
	if(matC_res3 != matC_ref3) {
		std::cout << "    -> Failure: Multiplication failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	
	// ********** Part D **********
	
	std::cout << "[D] divition (/)" << std::endl;
	// TODO
	
	
	// ********** Part E **********
	
	Matrix<3, 3, int> matE1_test;
	Matrix<3, 3, int> matE1_res;
	Matrix<3, 3, int> matE1_ref;
	Matrix<2, 3, int> matE2_test;
	Matrix<3, 2, int> matE2_res;
	Matrix<3, 2, int> matE2_ref;
	Matrix<3, 1, int> matE3_test;
	Matrix<1, 3, int> matE3_res;
	Matrix<1, 3, int> matE3_ref;
	
	std::cout << "[E] transpose()" << std::endl;
	
	std::cout << "    #" << testNo++ << ": 3x3 matrix" << std::endl;
	error = 0;
	
	matE1_test << 1, 2, 3,
	              4, 5, 6,
	              7, 8, 9;
	matE1_ref  << 1, 4, 7,
	              2, 5, 8,
	              3, 6, 9;
	matE1_res = matE1_test.transpose();
	
	if(matE1_res != matE1_ref) {
		std::cout << "    -> Failure: Transpose of 3x3 matrix failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": 2x3 matrix" << std::endl;
	error = 0;
	
	matE2_test << 1, 2, 3,
	              4, 5, 6;
	matE2_ref  << 1, 4,
	              2, 5,
	              3, 6;
	matE2_res = matE2_test.transpose();
	
	if(matE2_res != matE2_ref) {
		std::cout << "    -> Failure: Transpose of 2x3 matrix failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": 3x1 matrix" << std::endl;
	error = 0;
	
	matE3_test << 1,
	              2,
	              3;
	matE3_ref  << 1, 2, 3;
	matE3_res = matE3_test.transpose();
	
	if(matE3_res != matE3_ref) {
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
