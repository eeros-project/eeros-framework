#include <gtest/gtest.h>

#include <eeros/core/Fault.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros;
using namespace eeros::math;

// Testing constructors with initialization values
TEST(mathMatrixInitializationTest, init1) {
  Matrix<4, 2, int> iM4x2;

  for (unsigned int m = 0; m < 4; m++) {
    for (unsigned int n = 0; n < 2; n++) {
      iM4x2(m, n) = m + n;
    }
  }
  Matrix<4, 2, int> iM4x2copy(iM4x2);
  for (unsigned int m = 0; m < 4; m++) {
    for (unsigned int n = 0; n < 2; n++) {
      EXPECT_EQ(iM4x2copy(m, n), iM4x2(m, n));
    }
  }
}

// #include <iostream>
// #include <fstream>
// #include <eeros/math/Matrix.hpp>
// #include "../../Utils.hpp"
//
// #define DEFAULT_TOL 0.0001
//
// using namespace eeros::math;
//
// template < uint8_t M, uint8_t N, typename T >
// void print(Matrix<M, N, T> &A, int indent = 1) {
// 	for(int m = 0; m < M; m++) {
// 		for(int i = 0; i < indent; i++) std::cout << '\t';
// 		for(int n = 0; n < N; n++) {
// 			if(n > 0) std::cout << '\t';
// 			std::cout << A(m, n);
// 		}
// 		std::cout << std::endl;
// 	}
// }
//
// int main(int argc, char *argv[]) {
// 	int error = 0, errorSum = 0;
// 	int testNo = 1;
//
// 	Matrix<4, 2, int> iM4x2;
// 	Matrix<5, 5, int> iM5x5;
// 	Matrix<4, 2, double> dM4x2;
// 	Matrix<5, 5, double> dM5x5;
//
// 	int i;
// 	double d;
//
// 	std::cout << "Testing initialization methods for class Matrix" <<
// std::endl;
//
// 	// ********** Check parameters **********
// 	if(argc != 4) {
// 		std::cout << "-> Failure: illegal number of arguments!" <<
// std::endl; 		return -1;
// 	}
//
// 	// ********** Part A: Constructors with initialization values **********
//
// 	std::cout << "[A] Testing constructors with initialization values" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Copy constructor" << std::endl;
// 	error = 0;
// 	for(unsigned int m = 0; m < 4; m++) {
// 		for(unsigned int n = 0; n < 2; n++) {
// 			iM4x2(m, n) = m + n;
// 		}
// 	}
// 	Matrix<4, 2, int> iM4x2copy(iM4x2);
// 	for(unsigned int m = 0; m < 4; m++) {
// 		for(unsigned int n = 0; n < 2; n++) {
// 			if(iM4x2copy(m, n) != iM4x2(m, n)) {
// 				std::cout << "    -> Failure: M(" << m << ',' << n <<
// ") = " << iM4x2copy(m, n) << ", but should be " << iM4x2(m, n) << "!" <<
// std::endl; 				error++;
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
//
// 	std::cout << "    #" << testNo++ << ": Single value constructor" <<
// std::endl; 	error = 0; 	Matrix<8, 5, int> iM8x5(27);
// for(unsigned int m = 0; m
// < 8; m++) { 		for(unsigned int n = 0; n < 5; n++) {
// if(iM8x5(m, n) != 27) { 				std::cout << "    -> Failure: M(" << m << ',' << n <<
// ") = " << iM8x5(m, n) << ", but should be 27!" << std::endl;
// error++;
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
//
// 	std::cout << "    #" << testNo++ << ": Multiple value constructor (3x1)"
// << std::endl; 	error = 0; 	Matrix<3, 1, int> iM3x1(1, 2, 3);
// for(unsigned int m
// = 0; m < 3; m++) { 		if(iM3x1(m, 0) != m + 1) {
// std::cout << "    -> Failure: M(" << m << ", 0) = " << iM3x1(m, 0) << ", but
// should be " << m + 1 << "!" << std::endl; 			error++;
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 		std::cout << "    #" << testNo++ << ": Multiple value
// constructor
// (6x3)" << std::endl; 	error = 0; 	Matrix<6, 3, int> iM6x3(1, 2, 3, 4,
// 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18); 	i = 1; for(unsigned int
// n = 0; n < 3; n++) { 		for(unsigned int m = 0; m < 6; m++) {
// if(iM6x3(m, n) != i++) { 				std::cout << "    -> Failure: M(" << m << ", " << n
// << ") = " << iM6x3(m, n) << ", but should be " << i << "!" << std::endl;
// error++;
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
//
// 	std::cout << "    #" << testNo++ << ": Multiple value constructor (6x3)"
// << std::endl; 	error = 0; 	constexpr double c0 = -0.1; 	constexpr
// double c1 = 3.1415; 	constexpr double c2 = 10; 	Matrix<3, 1, double>
// dM3x1(c0, c1, c2); 	if(dM3x1(0, 0) != c0) { 		std::cout << "    -> Failure: M(0,
// 0) = " << dM3x1(0, 0) << ", but should be " << c0 << "!" << std::endl;
// error++;
// 	}
// 	if(dM3x1(1, 0) != c1) {
// 		std::cout << "    -> Failure: M(1, 0) = " << dM3x1(1, 0) << ",
// but should be " << c1 << "!" << std::endl; 		error++;
// 	}
// 	if(dM3x1(2, 0) != c2) {
// 		std::cout << "    -> Failure: M(2, 0) = " << dM3x1(2, 0) << ",
// but should be " << c2 << "!" << std::endl; 		error++;
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
//
// 	// ********** Part B: zero() and eye() **********
//
// 	std::cout << "[B] Testing zero() and eye() with integer and double
// matrices" << std::endl; 	iM5x5.eye(); 	dM5x5.eye(); iM4x2.zero();
// dM4x2.zero();
//
// 	std::cout << "    #" << testNo++ << ": Checking zero() with a 4x2
// integer matrix" << std::endl; 	error = 0; 	for(unsigned int m = 0; m
// < 4; m++)
// { 		for(unsigned int n = 0; n < 2; n++) { 			if(iM4x2(m,
// n) != 0) { 				std::cout << "
// -> Failure: M(" << m << ',' << n << ") = " << iM4x2(m, n) << ", but should be
// 0!" << std::endl; 				error++;
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Checking zero with a 4x2 double
// matrix" << std::endl; 	error = 0; 	for(unsigned int m = 0; m < 4;
// m++) { 		for(unsigned int n = 0; n < 2; n++) { 			if(dM4x2(m, n) != 0) { 				std::cout
// << "    -> Failure: M(" << m << ',' << n << ") = " << dM4x2(m, n) << ", but
// should be 0!" << std::endl; 				error++;
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Checking eye() with a 5x5 integer
// matrix" << std::endl; 	error = 0; 	for(unsigned int m = 0; m < 5;
// m++) { 		for(unsigned int n = 0; n < 5; n++) { 			if(m ==n) { 				if(iM5x5(m, n) != 1)
// { 					std::cout << "    -> Failure: M(" << m << ',' << n
// << ") = " << iM5x5(m, n) << ", but should be 1!" << std::endl;
// error++;
// 				}
// 			}
// 			else {
// 				if(iM5x5(m, n) != 0) {
// 					std::cout << "    -> Failure: M(" << m << ',' <<
// n
// << ") = " << iM5x5(m, n) << ", but should be 0!" << std::endl;
// error++;
// 				}
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Checking eye() with a 5x5 double
// matrix" << std::endl; 	error = 0; 	for(unsigned int m = 0; m < 5;
// m++) { 		for(unsigned int n = 0; n < 5; n++) { 			if(m ==n) { 				if(dM5x5(m, n) != 1)
// { 					std::cout << "    -> Failure: M(" << m << ',' << n
// << ") = " << dM5x5(m, n) << ", but should be 1!" << std::endl;
// error++;
// 				}
// 			}
// 			else {
// 				if(dM5x5(m, n) != 0) {
// 					std::cout << "    -> Failure: M(" << m << ',' <<
// n
// << ") = " << dM5x5(m, n) << ", but should be 0!" << std::endl;
// error++;
// 				}
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	// ********** Part C: rotX(), rotY(), rotZ() **********
//
// 	std::cout << "[C] Testing rotX(), rotX() and rotZ() with a 3x3 double
// matrix" << std::endl; 	eeros::math::Matrix<3, 3, double>
// referenceValues; 	eeros::math::Matrix<3, 3, double> calculatedValues; 	int line
// = 0;
//
// 	std::cout << "    #" << testNo++ << ": Checking rotX()" << std::endl;
// 	error = 0;
// 	std::ifstream fileX(argv[1]);
// 	if(!fileX.is_open()) {
// 		std::cout << "    -> Failure while opening file " << argv[1] <<
// std::endl; 		return -2;
// 	}
// 	while(!fileX.eof()) {
// 		line++;
// 		// read angle from file
// 		double angle;
// 		fileX >> angle;
//
// 		// read reference values from file
// 		for(int i = 0; i < 3 * 3; i++) {
// 			double in;
// 			fileX >> in;
// 			referenceValues(i) = in;
// 		}
// 		if(fileX.eof()) break;
//
// 		calculatedValues.rotx(angle);
//
// 		for(int n = 0; n < 3; n++) {
// 			for(int m = 0; m < 3; m++) {
// 				if(!Utils::compareApprox(referenceValues(m, n),
// calculatedValues(m, n), 0.001)) { 					std::cout << "
// -> Failure on line " << line << ": expecting " << referenceValues(m, n) << ",
// calculated " << calculatedValues(m, n) << std::endl;
// error++;
// 				}
// 			}
// 		}
// 	}
// 	fileX.close();
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 		std::cout << "    #" << testNo++ << ": Checking rotY()" <<
// std::endl; 	error = 0; 	std::ifstream fileY(argv[2]);
// if(!fileY.is_open()) { 		std::cout << "    -> Failure while opening file " <<
// argv[2] << std::endl; 		return -2;
// 	}
// 	line = 0;
// 	while(!fileY.eof()) {
// 		line++;
// 		// read angle from file
// 		double angle;
// 		fileY >> angle;
//
// 		// read reference values from file
// 		for(int i = 0; i < 3 * 3; i++) {
// 			double in;
// 			fileY >> in;
// 			referenceValues(i) = in;
// 		}
// 		if(fileY.eof()) break;
//
// 		calculatedValues.roty(angle);
//
// 		for(int n = 0; n < 3; n++) {
// 			for(int m = 0; m < 3; m++) {
// 				if(!Utils::compareApprox(referenceValues(m, n),
// calculatedValues(m, n), 0.001)) { 					std::cout << "
// -> Failure on line " << line << ": expecting " << referenceValues(m, n) << ",
// calculated " << calculatedValues(m, n) << std::endl;
// error++;
// 				}
// 			}
// 		}
// 	}
// 	fileY.close();
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Checking rotZ()" << std::endl;
// 	error = 0;
// 	std::ifstream fileZ(argv[3]);
// 	if(!fileZ.is_open()) {
// 		std::cout << "    -> Failure while opening file " << argv[3] <<
// std::endl; 		return -2;
// 	}
// 	line = 0;
// 	while(!fileZ.eof()) {
// 		line++;
// 		// read angle from file
// 		double angle;
// 		fileZ >> angle;
//
// 		// read reference values from file
// 		for(int i = 0; i < 3 * 3; i++) {
// 			double in;
// 			fileZ >> in;
// 			referenceValues(i) = in;
// 		}
// 		if(fileZ.eof()) break;
//
// 		calculatedValues.rotz(angle);
//
// 		for(int n = 0; n < 3; n++) {
// 			for(int m = 0; m < 3; m++) {
// 				if(!Utils::compareApprox(referenceValues(m, n),
// calculatedValues(m, n), 0.001)) { 					std::cout << "
// -> Failure on line " << line << ": expecting " << referenceValues(m, n) << ",
// calculated " << calculatedValues(m, n) << std::endl;
// error++;
// 				}
// 			}
// 		}
// 	}
// 	fileZ.close();
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
//
// 	std::cout << "[D] Testing other initialization methods" << std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Testing createDiag()" <<
// std::endl; 	error = 0; 	int diagV = 9; 	Matrix<5, 5, int> diagM =
// Matrix<5, 5, int>::createDiag(diagV); 	for(unsigned int m = 0; m < 5;
// m++) { 		for(unsigned
// int n = 0; n < 5; n++) { 			if(m == n) {
// if(diagM(m, n) != diagV) { 					std::cout << "
// -> Failure: M(" << m << ',' << n << ") = " << diagM(m, n) << ", but should be
// " << diagV << "!" << std::endl; 					error++;
// 				}
// 			}
// 			else {
// 				if(diagM(m, n) != 0) {
// 					std::cout << "    -> Failure: M(" << m << ',' <<
// n
// << ") = " << diagM(m, n) << ", but should be 0!" << std::endl;
// error++;
// 				}
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Testing createSkewSymmetric()" <<
// std::endl; 	error = 0; 	Matrix<3, 1, int> ssMatA, ssMatB;
// ssMatA(0) = 1;
// ssMatA(1) = 2; ssMatA(2) = 3; 	ssMatB(0) = -5; ssMatB(1) = -3; ssMatB(2)
// = 0; 	Matrix<3, 3, int> ssRefResA, ssRefResB, ssRes; 	ssRefResA(0, 0) =  0;
// ssRefResA(0, 1) = -3; ssRefResA(0, 2) =  2; 	ssRefResA(1, 0) =  3;
// ssRefResA(1, 1) =  0; ssRefResA(1, 2) = -1; 	ssRefResA(2, 0) = -2;
// ssRefResA(2, 1) =  1; ssRefResA(2, 2) =  0; 	ssRefResB(0, 0) =  0;
// ssRefResB(0, 1) =  0; ssRefResB(0, 2) = -3; 	ssRefResB(1, 0) =  0;
// ssRefResB(1, 1) =  0; ssRefResB(1, 2) =  5; 	ssRefResB(2, 0) =  3;
// ssRefResB(2, 1) = -5; ssRefResB(2, 2) =  0; 	ssRes = Matrix<3, 3,
// int>::createSkewSymmetric(ssMatA); 	for(unsigned int m = 0; m < 3; m++) {
// 		for(unsigned int n = 0; n < 3; n++) {
// 			if(ssRes(m, n) != ssRefResA(m, n)) {
// 				std::cout << "    -> Failure: M(" << m << ',' << n <<
// ") = " << ssRes(m, n) << ", but should be " << ssRefResA(m, n) << "!" <<
// std::endl; 				error++;
// 			}
// 		}
// 	}
// 	ssRes = Matrix<3, 3, int>::createSkewSymmetric(ssMatB);
// 	for(unsigned int m = 0; m < 3; m++) {
// 		for(unsigned int n = 0; n < 3; n++) {
// 			if(ssRes(m, n) != ssRefResB(m, n)) {
// 				std::cout << "    -> Failure: M(" << m << ',' << n <<
// ") = " << ssRes(m, n) << ", but should be " << ssRefResB(m, n) << "!" <<
// std::endl; 				error++;
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Testing << operator with a 3x3
// integer matrix" << std::endl; 	error = 0; 	Matrix<3, 3, int> sMat;
// sMat << 1,
// 4, 7, 	        2, 5, 8, 	        3, 6, 9; 	int j = 1;
// for(unsigned int n = 0; n < 3; n++) { 		for(unsigned int m = 0; m < 3; m++) {
// 			if(j != sMat(m, n)) {
// 				std::cout << "    -> Failure: M(" << m << ',' << n <<
// ") = " << sMat(m, n) << ", but should be " << j << '!' << std::endl;
// error++;
// 			}
// 			j++;
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Testing << operator with a 4x1
// double matrix" << std::endl; 	error = 0; 	Matrix<4, 1, double> tMat;
// tMat << 0.1, 0.2, 0.3, 0.4; 	double l = 0.1; 	for(unsigned int m = 0;
// m < 4; m++) { 		if(!Utils::compareApprox(l, tMat(m, 0), DEFAULT_TOL)) {
// 			std::cout << "    -> Failure: M(" << m << ", 0) = " <<
// tMat(m, 0) << ", but should be " << l << '!' << std::endl;
// error++;
// 		}
// 		l += 0.1;
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Testing = operator with an
// integer for a integer matrix" << std::endl; 	error = 0; 	Matrix<3, 3,
// int> aMat1; 	for(int k = -4; k < 5; k++) { 		aMat1 = k;
// for(unsigned int n = 0; n <
// 3; n++) { 			for(unsigned int m = 0; m < 3; m++) {
// if(aMat1(m, n) != k) { 					std::cout << "    -> Failure: M(" << m << ',' << n
// << ") = " << aMat1(m, n) << ", but should be " << k << '!' << std::endl;
// 					error++;
// 				}
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	std::cout << "    #" << testNo++ << ": Testing = operator with an
// integer for a double matrix" << std::endl; 	error = 0; 	Matrix<3, 3,
// double> aMat2; 	for(int k = -4; k < 5; k++) { 		aMat2 = k;
// for(unsigned int n = 0; n <
// 3; n++) { 			for(unsigned int m = 0; m < 3; m++) {
// if(aMat2(m, n) != k) { 					std::cout << "    -> Failure: M(" << m << ',' << n
// << ") = " << aMat2(m, n) << ", but should be " << k << '!' << std::endl;
// 					error++;
// 				}
// 			}
// 		}
// 	}
// 	errorSum += error;
// 	std::cout << "    -> Test finished with " << error << " error(s)" <<
// std::endl;
//
// 	if (errorSum == 0)
// 		std::cout << "Matrix initialization test succeeded" <<
// std::endl; 	else 		std::cout << "Matrix initialization test failed
// with " << errorSum << " error(s)" << std::endl;
//
// 	return errorSum;
// }
