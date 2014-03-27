#include <eeros/math/MatrixIndexOutOfBoundException.hpp>
#include <sstream>

using namespace eeros::math;

MatrixIndexOutOfBoundException::MatrixIndexOutOfBoundException(unsigned int m, unsigned int M, unsigned int n, unsigned int N) {
	std::stringstream ss;
	ss << "Index out of bound (m = " << m << ", M = " << M << ", n = " << n << ", N = " << N << ")";
	message = ss.str();
}

MatrixIndexOutOfBoundException::~MatrixIndexOutOfBoundException() throw() { }
