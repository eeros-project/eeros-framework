#include <cstdlib>
#include <iostream>
#include <cmath>

#include <eeros/math/Matrix.hpp>
using namespace eeros::math;

template < uint8_t N, uint8_t M, typename T >
void print(Matrix<N,M,T> &A, int indent = 1) {
	for(int n = 0; n < N; n++) {
		for(int i = 0; i < indent; i++) std::cout << '\t';
		for(int m = 0; m < M; m++) {
			if(m > 0) std::cout << '\t';
			std::cout << A(n,m);
		}
		std::cout << std::endl;
	}
}

template < uint8_t N, uint8_t M, typename T >
void rot(int axis, Matrix<N,M,T> &A, T angle) {
	if (axis == 0)
		A.rotx(angle);
	else if (axis == 1)
		A.roty(angle);
	else
		A.rotz(angle);
}

int main(int argc, char *argv[]) {
	int error = 0;
	int testNo = 0;

	/********** A) Creating and inizializing **********/
	
	// Element access
	testNo++;
	std::cout << "Test #" << testNo << ": Element access" << std::endl;
	Matrix<3,3> m123;
	m123.zero();
	int i = 1;
	for(int n = 0; n < 3; n++) {
		for(int m = 0; m < 3; m++) {
			m123(m,n) = i++;
		}
	}
	i = 1;
	for(int n = 0; n < 3; n++) {
		for(int m = 0; m < 3; m++) {
			if(i++ != m123(m,n)) {
				std::cout << "  Failure: M(" << m << ',' << n << ") = " << m123(m,n) << ", but should be " << i << '!' << std::endl;
				error++;
			}
		}
	}
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// zero()
	testNo++;
	std::cout << "Test #" << testNo << ": zero()" << std::endl;
	Matrix<3,3> mZero;
	mZero.zero();
	for(int n = 0; n < 3; n++) {
		for(int m = 0; m < 3; m++) {
			if(mZero(n,m) != 0) {
				std::cout << "  Failure: M(" << m << ',' << n << ") = " << mZero(m,n) << ", but should be 0!" << std::endl;
				error++;
			}
		}
	}
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// eye()
 	testNo++;
	std::cout << "Test #" << testNo << ": eye()" << std::endl;
	Matrix<3,3> mEye;
	mEye.eye();
	for(int n = 0; n < 3; n++) {
		for(int m = 0; m < 3; m++) {
			if(n == m) { // value should be 1
				if(mEye(n,m) != 1) {
					std::cout << "  Failure: M(" << m << ',' << n << ") = " << mEye(m,n) << ", but should be 1!" << std::endl;
					error++;
				}
			}
			else { // value should be 0
				if(mEye(n,m) != 0) {
					std::cout << "  Failure: M(" << m << ',' << n << ") = " << mEye(m,n) << ", but should be 0!" << std::endl;
					error++;
				}
			}
		}
	}
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// rotx()
 	testNo++;
	std::cout << "Test #" << testNo << ": rotx()" << std::endl;
	Matrix<3,3> mRotx;
	double rotxAngle = M_PI;
	mRotx.rotx(rotxAngle);
	// TODO
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// roty()
 	testNo++;
	std::cout << "Test #" << testNo << ": roty()" << std::endl;
	Matrix<3,3> mRoty;
	double rotyAngle = M_PI;
	mRoty.roty(rotyAngle);
	// TODO
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// rotz()
 	testNo++;
	std::cout << "Test #" << testNo << ": rotz()" << std::endl;
	Matrix<3,3> mRotz;
	double rotzAngle = M_PI;
	mRotz.rotz(rotzAngle);
	// TODO
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	/********** Functions for checking the matrix characteristics **********/
	
	// TODO
	
	/********** Functions for calculating some characteristics of the matrix **********/
	
	
	
// 	double m2trace = m2.trace();
// 	if (m2trace != 3) {
// 		std::cout << stage << ": trace expected to be 3, m2.trace() = " << m2trace << std::endl;
// 		error++;
// 	}
// 	for(int n = 0; n < 3; n++) {
// 		for(int m = 0; m < 3; m++) {
// 			if(n == m) {
// 				if(m2(n,m) != 1) {
// 					std::cout << stage << ": 1 expected, n = " << n << ", m = " << m << std::endl;
// 					error++;
// 				}
// 			}
// 			else {
// 				if (m2(n,m) != 0) {
// 					std::cout << stage << ": 0 expected, n = " << n << ", m = " << m << std::endl;
// 					error++;
// 				}
// 			}
// 		}
// 	}
// 	if (m1 == m2) {
// 		std::cout << stage << ": == fails" << std::endl;
// 		error++;
// 	}
// 	if (!(m1 != m2)) {
// 		std::cout << stage << ": != fails" << std::endl;
// 		error++;
// 	}
// 	std::cout << "Stage " << stage << ":" << " m2 = " << std::endl;
// 	print(m2);
// 
// 	// Stage 3: +
// 	stage = 3;
// 	Matrix<3,3> m3 = m1 + m2;
// 	if (m3 != m2) {
// 		std::cout << stage << ": eye expected" << std::endl;
// 		error++;
// 	}
// 	std::cout << "Stage " << stage << ":" << " m3 = m1 + m2 = " << std::endl;
// 	print(m3);

	// Stage 4: rot()
// 	stage = 4;
// 	Matrix<3,3> m4;
// 	for(int j = 0; j < 3; j++) {
// 		for(int i = 0; i < 360; i++) {
// 			rot<3,3,double>(j, m4, i*3.14/180);
// 			Matrix<3,3> m4inv = !m4;
// 			Matrix<3,3> m4result = (m4 * m4inv);
// 
// 			double sum = 0;
// 			for(int n = 0; n < 3; n++) {
// 				for (int m = 0; m < 3; m++) {
// 					double res = m4result(n,m);
// 					sum += (res > 0 ) ? res : -res;
// 				}
// 			}
// 
// 			if(sum != 3) {
// 				std::cout << stage << ": inverse fails, j = " << j << ", i = " << i << ", sum = " << sum << std::endl;
// 				error++;
// 
// 				std::cout << "  m4 " << std::endl;
// 				print(m4);
// 
// 				std::cout << "  m4inv =" << std::endl;
// 				print(m4inv);
// 
// 				std::cout << "  m4result =" << std::endl;
// 				print(m4result);
// 				break;
// 			}
// 		}
// 	}
// 	std::cout << "Stage " << stage << ":" << " m4 = " << std::endl;
// 	print(m4);
// 
// 	// Stage 4: sizeof()
// 	stage = 5;
// 	Matrix<2,4> m5;
// 	std::cout << "Stage " << stage << ":" << " Size of m5 (Matrix<2,4,double>) = " << sizeof(m5) << std::endl;
	
	if (error == 0)
		std::cout << "matrix test succeeded" << std::endl;
	else
		std::cout << "matrix test failed: " << error << " errors" << std::endl;

	return error;
}
