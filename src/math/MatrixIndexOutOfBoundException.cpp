#include <eeros/math/MatrixIndexOutOfBoundException.hpp>
#include <sstream>

using namespace eeros::math;

MatrixIndexOutOfBoundException::MatrixIndexOutOfBoundException(unsigned int i, unsigned int I) {
	std::stringstream ss;
	ss << "Index out of bound (i = " << i << ", I = " << I << ")";
	message = ss.str();
}

MatrixIndexOutOfBoundException::MatrixIndexOutOfBoundException(unsigned int m, unsigned int M, unsigned int n, unsigned int N) {
	std::stringstream ss;
	ss << "Index out of bound (m = " << m << ", M = " << M << ", n = " << n << ", N = " << N << ")";
	message = ss.str();
}

MatrixIndexOutOfBoundException::MatrixIndexOutOfBoundException(unsigned int m, unsigned int M, unsigned int n, unsigned int N, std::string postfix) {
	std::stringstream ss;
	ss << "Index out of bound (m = " << m << ", M = " << M << ", n = " << n << ", N = " << N << ") " << postfix;
	message = ss.str();
}

MatrixIndexOutOfBoundException::~MatrixIndexOutOfBoundException() throw() { }
