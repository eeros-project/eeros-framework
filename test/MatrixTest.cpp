#include <cstdlib>
#include <iostream>

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
	int stage = 0;

	// Stage 1: zero()
	stage = 1;
	Matrix<3,3> m1;
	m1.zero();
	for(int n = 0; n < 3; n++) {
		for(int m = 0; m < 3; m++) {
			if(m1(n,m) != 0) {
				std::cout << stage << ": 0 expected, n = " << n << ", m = " << m << std::endl;
				error++;
			}
		}
	}
	std::cout << "Stage " << stage << ":" << " m1 = " << std::endl;
	print(m1);

	// Stage 2: eye()
	stage = 2;
	Matrix<3,3> m2;
	m2.eye();
	double m2trace = m2.trace();
	if (m2trace != 3) {
		std::cout << stage << ": trace expected to be 3, m2.trace() = " << m2trace << std::endl;
		error++;
	}
	for(int n = 0; n < 3; n++) {
		for(int m = 0; m < 3; m++) {
			if(n == m) {
				if(m2(n,m) != 1) {
					std::cout << stage << ": 1 expected, n = " << n << ", m = " << m << std::endl;
					error++;
				}
			}
			else {
				if (m2(n,m) != 0) {
					std::cout << stage << ": 0 expected, n = " << n << ", m = " << m << std::endl;
					error++;
				}
			}
		}
	}
	if (m1 == m2) {
		std::cout << stage << ": == fails" << std::endl;
		error++;
	}
	if (!(m1 != m2)) {
		std::cout << stage << ": != fails" << std::endl;
		error++;
	}
	std::cout << "Stage " << stage << ":" << " m2 = " << std::endl;
	print(m2);

	// Stage 3: +
	stage = 3;
	Matrix<3,3> m3 = m1 + m2;
	if (m3 != m2) {
		std::cout << stage << ": eye expected" << std::endl;
		error++;
	}
	std::cout << "Stage " << stage << ":" << " m3 = m1 + m2 = " << std::endl;
	print(m3);

	// Stage 4: rot()
	stage = 4;
	Matrix<3,3> m4;
	for(int j = 0; j < 3; j++) {
		for(int i = 0; i < 360; i++) {
			rot<3,3,double>(j, m4, i*3.14/180);
			Matrix<3,3> m4inv = !m4;
			Matrix<3,3> m4result = (m4 * m4inv);

			double sum = 0;
			for(int n = 0; n < 3; n++) {
				for (int m = 0; m < 3; m++) {
					double res = m4result(n,m);
					sum += (res > 0 ) ? res : -res;
				}
			}

			if(sum != 3) {
				std::cout << stage << ": inverse fails, j = " << j << ", i = " << i << ", sum = " << sum << std::endl;
				error++;

				std::cout << "  m4 " << std::endl;
				print(m4);

				std::cout << "  m4inv =" << std::endl;
				print(m4inv);

				std::cout << "  m4result =" << std::endl;
				print(m4result);
				break;
			}
		}
	}
	std::cout << "Stage " << stage << ":" << " m4 = " << std::endl;
	print(m4);

	// Stage 4: sizeof()
	stage = 5;
	Matrix<2,4> m5;
	std::cout << "Stage " << stage << ":" << " Size of m5 (Matrix<2,4,double>) = " << sizeof(m5) << std::endl;
	
	if (error == 0)
		std::cout << "matrix test succeeded" << std::endl;
	else
		std::cout << "matrix test failed: " << error << " errors" << std::endl;

	return error;
}
