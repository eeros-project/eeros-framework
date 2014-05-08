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
	
	Matrix<3, 3, int> matZeros1, matZeros2, matOnes, matTwos, matThrees, matSingleOne, matNegative;
	
	matZeros1.zero();
	matZeros2.zero();
	matOnes      <<  1,  1,  1,  1,  1,  1;
	matTwos      <<  2,  2,  2,  2,  2,  2;
	matThrees    <<  3,  3,  3,  3,  3,  3;
	matSingleOne <<  0,  0,  0,  1,  0,  0;
	matNegative  << -1, -1, -1, -1, -1, -1;
	
	std::cout << "Testing relational operators for class Matrix" << std::endl;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "[A] Testing operator '=='" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing two zero matrices" << std::endl;
	error = 0;
	if(!(matZeros1 == matZeros2)) {
		std::cout << "    -> Failure: comparision is negative, but should be positive" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing a two complete different matrices" << std::endl;
	error = 0;
	if(matOnes == matTwos) {
		std::cout << "    -> Failure: comparision is positive, but should be negative" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing a two matrices, where only one element is different" << std::endl;
	error = 0;
	if(matZeros1 == matSingleOne) {
		std::cout << "    -> Failure: comparision is positive, but should be negative" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing two matrices where the elements have only a different sign" << std::endl;
	error = 0;
	if(matOnes == matNegative) {
		std::cout << "    -> Failure: comparision is positive, but should be negative" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "[B] Testing operator '!='" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing two zero matrices" << std::endl;
	error = 0;
	if(matZeros1 != matZeros2) {
		std::cout << "    -> Failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing a two matrices, where only one element is different" << std::endl;
	error = 0;
	if(!(matZeros1 != matSingleOne)) {
		std::cout << "    -> Failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "[C] Testing operator '<'" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing two zero matrices" << std::endl;
	error = 0;
	if(matZeros1 < matZeros2) {
		std::cout << "    -> Failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing a zeros matrix with a ones matrix" << std::endl;
	error = 0;
	if(matOnes < matZeros1) {
		std::cout << "    -> Failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "    #" << testNo++ << ": Comparing a zeros matrix with a matrix with a single one" << std::endl;
	error = 0;
	if(matOnes < matSingleOne) {
		std::cout << "    -> Failed!" << std::endl;
		error++;
	}
	errorSum += error;
	std::cout << "    -> Test finished with " << error << " error(s)" << std::endl;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "[D] Testing operator '<='" << std::endl;
	// TODO
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "[E] Testing operator '>'" << std::endl;
	// TODO
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "[F] Testing operator '>='" << std::endl;
	// TODO
	
	if(errorSum == 0) {
		std::cout << "Matrix element access test succeeded" << std::endl;
	}
	else {
		std::cout << "Matrix element access test failed with " << errorSum << " error(s)" << std::endl;
	}
	
	return errorSum;
}
