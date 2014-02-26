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
	
	testNo++;
	std::cout << "Test #" << testNo << ": swaping Rows" << std::endl;
	
	Matrix<3,3> swapMatrix1;
	swapMatrix1(0,0) = 1; swapMatrix1(0,1) = 1; swapMatrix1(0,2) = 0;
	swapMatrix1(1,0) = 0; swapMatrix1(1,1) = 0; swapMatrix1(1,2) = 2;
	swapMatrix1(2,0) = 1; swapMatrix1(2,1) = 0; swapMatrix1(2,2) = 0;
	Matrix<3,3> swapSolutionMatrix1;
	swapSolutionMatrix1(0,0) = 0; swapSolutionMatrix1(0,1) = 0; swapSolutionMatrix1(0,2) = 2;
	swapSolutionMatrix1(1,0) = 1; swapSolutionMatrix1(1,1) = 1; swapSolutionMatrix1(1,2) = 0;
	swapSolutionMatrix1(2,0) = 1; swapSolutionMatrix1(2,1) = 0; swapSolutionMatrix1(2,2) = 0;
	
	
	Matrix<4,3> swapMatrix2;
	swapMatrix2(0,0) = 1; swapMatrix2(0,1) = 1; swapMatrix2(0,2) = 0;
	swapMatrix2(1,0) = 0; swapMatrix2(1,1) = 0; swapMatrix2(1,2) = 2;
	swapMatrix2(2,0) = 1; swapMatrix2(2,1) = 0; swapMatrix2(2,2) = 0;
	swapMatrix2(3,0) = 4; swapMatrix2(3,1) = 5; swapMatrix2(3,2) = 6;
	
	Matrix<4,3> swapSolutionMatrix2;
	swapSolutionMatrix2(0,0) = 4; swapSolutionMatrix2(0,1) = 5; swapSolutionMatrix2(0,2) = 6;
	swapSolutionMatrix2(1,0) = 0; swapSolutionMatrix2(1,1) = 0; swapSolutionMatrix2(1,2) = 2;
	swapSolutionMatrix2(2,0) = 1; swapSolutionMatrix2(2,1) = 0; swapSolutionMatrix2(2,2) = 0;
	swapSolutionMatrix2(3,0) = 1; swapSolutionMatrix2(3,1) = 1; swapSolutionMatrix2(3,2) = 0;
	
	
	
	
	swapMatrix1.swapRows(0,1);
	swapMatrix2.swapRows(0,3);
	
	if (swapMatrix1 != swapSolutionMatrix1){
	    std::cout << "  Failure: Matrix 1 rows not correctly swaped" << std::endl;
	    error++;
	}
	
	if (swapMatrix2 != swapSolutionMatrix2){
	    std::cout << "  Failure: Matrix 2 rows not correctly swaped" << std::endl;
	    error++;
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	
	
	testNo++;
	std::cout << "Test #" << testNo << ": Gauss sorting Matrix" << std::endl;
	Matrix<3,3> sortMatrix1;
	sortMatrix1(0,0) = 1; sortMatrix1(0,1) = 1; sortMatrix1(0,2) = 0;
	sortMatrix1(1,0) = 0; sortMatrix1(1,1) = 0; sortMatrix1(1,2) = 2;
	sortMatrix1(2,0) = 1; sortMatrix1(2,1) = 0; sortMatrix1(2,2) = 0;
	Matrix<3,3> sortSolutionMatrix1;
	sortSolutionMatrix1(0,0) = 1; sortSolutionMatrix1(0,1) = 1; sortSolutionMatrix1(0,2) = 0;
	sortSolutionMatrix1(1,0) = 1; sortSolutionMatrix1(1,1) = 0; sortSolutionMatrix1(1,2) = 0;
	sortSolutionMatrix1(2,0) = 0; sortSolutionMatrix1(2,1) = 0; sortSolutionMatrix1(2,2) = 2;
	
	Matrix<3,3> sortMatrix2;
	sortMatrix2(0,0) = 0; sortMatrix2(0,1) = 1; sortMatrix2(0,2) = 0;
	sortMatrix2(1,0) = 0; sortMatrix2(1,1) = 0; sortMatrix2(1,2) = 2;
	sortMatrix2(2,0) = 1; sortMatrix2(2,1) = 0; sortMatrix2(2,2) = 0;

	Matrix<3,3> sortSolutionMatrix2;
	sortSolutionMatrix2(0,0) = 1; sortSolutionMatrix2(0,1) = 0; sortSolutionMatrix2(0,2) = 0;
	sortSolutionMatrix2(1,0) = 0; sortSolutionMatrix2(1,1) = 1; sortSolutionMatrix2(1,2) = 0;
	sortSolutionMatrix2(2,0) = 0; sortSolutionMatrix2(2,1) = 0; sortSolutionMatrix2(2,2) = 2;
	
	
	Matrix<3,3> sortMatrix3;
	sortMatrix3(0,0) = 5; sortMatrix3(0,1) = 1; sortMatrix3(0,2) = 3;
	sortMatrix3(1,0) = 2; sortMatrix3(1,1) = 1; sortMatrix3(1,2) = 2;
	sortMatrix3(2,0) = 5.25; sortMatrix3(2,1) = 3.125; sortMatrix3(2,2) = 2.5;

	Matrix<3,3> sortSolutionMatrix3;
	sortSolutionMatrix3(0,0) = 5; sortSolutionMatrix3(0,1) = 1; sortSolutionMatrix3(0,2) = 3;
	sortSolutionMatrix3(1,0) = 2; sortSolutionMatrix3(1,1) = 1; sortSolutionMatrix3(1,2) = 2;
	sortSolutionMatrix3(2,0) = 5.25; sortSolutionMatrix3(2,1) = 3.125; sortSolutionMatrix3(2,2) = 2.5;
	
	Matrix<4,3> sortMatrix4;
	sortMatrix4(0,0) = 1; sortMatrix4(0,1) = 1; sortMatrix4(0,2) = 0;
	sortMatrix4(1,0) = 0; sortMatrix4(1,1) = 0; sortMatrix4(1,2) = 2;
	sortMatrix4(2,0) = 1; sortMatrix4(2,1) = 0; sortMatrix4(2,2) = 0;
	sortMatrix4(3,0) = 4; sortMatrix4(3,1) = 5; sortMatrix4(3,2) = 6;
	
	Matrix<4,3> sortSolutionMatrix4;
	sortSolutionMatrix4(0,0) = 1; sortSolutionMatrix4(0,1) = 1; sortSolutionMatrix4(0,2) = 0;
	sortSolutionMatrix4(1,0) = 1; sortSolutionMatrix4(1,1) = 0; sortSolutionMatrix4(1,2) = 0;
	sortSolutionMatrix4(2,0) = 4; sortSolutionMatrix4(2,1) = 5; sortSolutionMatrix4(2,2) = 6;
	sortSolutionMatrix4(3,0) = 0; sortSolutionMatrix4(3,1) = 0; sortSolutionMatrix4(3,2) = 2;
	
	Matrix<3,4> sortMatrix5;
	sortMatrix5(0,0) = 1; sortMatrix5(0,1) = 1; sortMatrix5(0,2) = 0; sortMatrix5(0,3) = 0;
	sortMatrix5(1,0) = 0; sortMatrix5(1,1) = 0; sortMatrix5(1,2) = 2; sortMatrix5(1,3) = 2;
	sortMatrix5(2,0) = 1; sortMatrix5(2,1) = 0; sortMatrix5(2,2) = 0; sortMatrix5(2,3) = 0;
	
	Matrix<3,4> sortSolutionMatrix5;
	sortSolutionMatrix5(0,0) = 1; sortSolutionMatrix5(0,1) = 1; sortSolutionMatrix5(0,2) = 0; sortSolutionMatrix5(0,3) = 0;
	sortSolutionMatrix5(1,0) = 1; sortSolutionMatrix5(1,1) = 0; sortSolutionMatrix5(1,2) = 0; sortSolutionMatrix5(1,3) = 0;
	sortSolutionMatrix5(2,0) = 0; sortSolutionMatrix5(2,1) = 0; sortSolutionMatrix5(2,2) = 2; sortSolutionMatrix5(2,3) = 2;
	
	Matrix<3,4> sortMatrix6;
	sortMatrix6(0,0) = 2; sortMatrix6(0,1) = 4; sortMatrix6(0,2) = 6; sortMatrix6(0,3) = 8;
	sortMatrix6(1,0) = 1; sortMatrix6(1,1) = 2; sortMatrix6(1,2) = 3; sortMatrix6(1,3) = 4;
	sortMatrix6(2,0) = 0; sortMatrix6(2,1) = 1; sortMatrix6(2,2) = 1; sortMatrix6(2,3) = 1;
	
	Matrix<3,4> sortSolutionMatrix6;
	sortSolutionMatrix6(0,0) = 2; sortSolutionMatrix6(0,1) = 4; sortSolutionMatrix6(0,2) = 6; sortSolutionMatrix6(0,3) = 8;
	sortSolutionMatrix6(1,0) = 1; sortSolutionMatrix6(1,1) = 2; sortSolutionMatrix6(1,2) = 3; sortSolutionMatrix6(1,3) = 4;
	sortSolutionMatrix6(2,0) = 0; sortSolutionMatrix6(2,1) = 1; sortSolutionMatrix6(2,2) = 1; sortSolutionMatrix6(2,3) = 1;
	
	
	sortMatrix1.sortForGaussAlgorithm();
	sortMatrix2.sortForGaussAlgorithm();
	sortMatrix3.sortForGaussAlgorithm();
	sortMatrix4.sortForGaussAlgorithm();
	sortMatrix5.sortForGaussAlgorithm();
	sortMatrix6.sortForGaussAlgorithm();
	
	
	if (sortMatrix1 != sortSolutionMatrix1){
	    std::cout << "  Failure: Matrix 1 not sorted correctly" << std::endl;
	    error++;
	}
	
	if (sortMatrix2 != sortSolutionMatrix2){
	    std::cout << "  Failure: Matrix 2 not sorted correctly" << std::endl;
	    error++;
	}
	
	if (sortMatrix3 != sortSolutionMatrix3){
	    std::cout << "  Failure: Matrix 3 not sorted correctly" << std::endl;
	    error++;
	}
	if (sortMatrix4 != sortSolutionMatrix4){
	    std::cout << "  Failure: Matrix 4 not sorted correctly" << std::endl;
	    error++;
	}
	if (sortMatrix5 != sortSolutionMatrix5){
	    std::cout << "  Failure: Matrix 5 not sorted correctly" << std::endl;
	    error++;
	}
	
	if (sortMatrix6 != sortSolutionMatrix6){
	    std::cout << "  Failure: Matrix 6 not sorted correctly" << std::endl;
	    error++;
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	
	
	testNo++;
	std::cout << "Test #" << testNo << ": gaus row elimination" << std::endl;
	
	Matrix<3,3> gaussMatrix1;
	gaussMatrix1(0,0) = 1; gaussMatrix1(0,1) = 1; gaussMatrix1(0,2) = 0;
	gaussMatrix1(1,0) = 0; gaussMatrix1(1,1) = 0; gaussMatrix1(1,2) = 2;
	gaussMatrix1(2,0) = 1; gaussMatrix1(2,1) = 0; gaussMatrix1(2,2) = 0;

	Matrix<3,3> gaussSolutionMatrix1;
	gaussSolutionMatrix1(0,0) = 1; gaussSolutionMatrix1(0,1) = 1; gaussSolutionMatrix1(0,2) = 0;
	gaussSolutionMatrix1(1,0) = 0; gaussSolutionMatrix1(1,1) = -1; gaussSolutionMatrix1(1,2) = 0;
	gaussSolutionMatrix1(2,0) = 0; gaussSolutionMatrix1(2,1) = 0; gaussSolutionMatrix1(2,2) = 2;
	
	
	Matrix<3,3> gaussMatrix2;
	gaussMatrix2(0,0) = 1; gaussMatrix2(0,1) = 3; gaussMatrix2(0,2) = 2;
	gaussMatrix2(1,0) = 2; gaussMatrix2(1,1) = 4; gaussMatrix2(1,2) = 4;
	gaussMatrix2(2,0) = 3; gaussMatrix2(2,1) = 5; gaussMatrix2(2,2) = 6;
	
	
	Matrix<3,3> gaussSolutionMatrix2;
	gaussSolutionMatrix2(0,0) = 1; gaussSolutionMatrix2(0,1) = 3; gaussSolutionMatrix2(0,2) = 2;
	gaussSolutionMatrix2(1,0) = 0; gaussSolutionMatrix2(1,1) = -2; gaussSolutionMatrix2(1,2) = 0;
	gaussSolutionMatrix2(2,0) = 0; gaussSolutionMatrix2(2,1) = 0; gaussSolutionMatrix2(2,2) = 0;
	
	
	Matrix<3,4> gaussMatrix3;
	gaussMatrix3(0,0) = 1; gaussMatrix3(0,1) = 2; gaussMatrix3(0,2) = 3; gaussMatrix3(0,3) = 4;
	gaussMatrix3(1,0) = -1; gaussMatrix3(1,1) = 0; gaussMatrix3(1,2) = 1; gaussMatrix3(1,3) = 0;
	gaussMatrix3(2,0) = 3; gaussMatrix3(2,1) = 5; gaussMatrix3(2,2) = 6; gaussMatrix3(2,3) = 9;
	
	Matrix<3,4> gaussSolutionMatrix3;
	gaussSolutionMatrix3(0,0) = 1; gaussSolutionMatrix3(0,1) = 2; gaussSolutionMatrix3(0,2) = 3; gaussSolutionMatrix3(0,3) = 4;
	gaussSolutionMatrix3(1,0) = 0; gaussSolutionMatrix3(1,1) = 2; gaussSolutionMatrix3(1,2) = 4; gaussSolutionMatrix3(1,3) = 4;
	gaussSolutionMatrix3(2,0) = 0; gaussSolutionMatrix3(2,1) = 0; gaussSolutionMatrix3(2,2) = -1; gaussSolutionMatrix3(2,3) = -1;
	
	
	Matrix<3,4> gaussMatrix4;
	gaussMatrix4(0,0) = 2; gaussMatrix4(0,1) = 4; gaussMatrix4(0,2) = 6; gaussMatrix4(0,3) = 8;
	gaussMatrix4(1,0) = 1; gaussMatrix4(1,1) = 2; gaussMatrix4(1,2) = 3; gaussMatrix4(1,3) = 5;
	gaussMatrix4(2,0) = 0; gaussMatrix4(2,1) = 1; gaussMatrix4(2,2) = 1; gaussMatrix4(2,3) = 1;
	
	Matrix<3,4> gaussSolutionMatrix4;
	gaussSolutionMatrix4(0,0) = 2; gaussSolutionMatrix4(0,1) = 4; gaussSolutionMatrix4(0,2) = 6; gaussSolutionMatrix4(0,3) = 8;
	gaussSolutionMatrix4(1,0) = 0; gaussSolutionMatrix4(1,1) = 0; gaussSolutionMatrix4(1,2) = 0; gaussSolutionMatrix4(1,3) = 1;
	gaussSolutionMatrix4(2,0) = 0; gaussSolutionMatrix4(2,1) = 1; gaussSolutionMatrix4(2,2) = 1; gaussSolutionMatrix4(2,3) = 1;
	
	
	gaussMatrix1.gaussRowElimination();
	gaussMatrix2.gaussRowElimination();
	gaussMatrix3.gaussRowElimination();
	gaussMatrix4.gaussRowElimination();
	 
	
	if (gaussMatrix1 != gaussSolutionMatrix1){
	    std::cout << "  Failure: Matrix 1 row elimination not correctly done" << std::endl;
	    error++;
	}
	
	if (gaussMatrix2 != gaussSolutionMatrix2){
	    std::cout << "  Failure: Matrix 2 row elimination not correctly done" << std::endl;
	    error++;
	}
	
	if (gaussMatrix3 != gaussSolutionMatrix3){
	    std::cout << "  Failure: Matrix 3 row elimination not correctly done" << std::endl;
	    error++;
	}
	
	if (gaussMatrix4 != gaussSolutionMatrix4){
	    std::cout << "  Failure: Matrix 4 row elimination not correctly done" << std::endl;
	    error++;
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	
	
	
	
	/********** Functions for checking the matrix characteristics **********/
	
	Matrix<3,3> characteristicsMatrix1;
	characteristicsMatrix1(0,0) = 1; characteristicsMatrix1(0,1) = 0; characteristicsMatrix1(0,2) = 0;
	characteristicsMatrix1(1,0) = 0; characteristicsMatrix1(1,1) = 1; characteristicsMatrix1(1,2) = 0;
	characteristicsMatrix1(2,0) = 0; characteristicsMatrix1(2,1) = 0; characteristicsMatrix1(2,2) = 1;
	
	Matrix<3,3> characteristicsMatrix2;
	characteristicsMatrix2(0,0) = 1; characteristicsMatrix2(0,1) = 3; characteristicsMatrix2(0,2) = 2;
	characteristicsMatrix2(1,0) = 2; characteristicsMatrix2(1,1) = 4; characteristicsMatrix2(1,2) = 4;
	characteristicsMatrix2(2,0) = 3; characteristicsMatrix2(2,1) = 5; characteristicsMatrix2(2,2) = 6;
	
	Matrix<2,2> characteristicsMatrix3;
	characteristicsMatrix3(0,0) = 2; characteristicsMatrix3(0,1) = 5; 
	characteristicsMatrix3(1,0) = 4; characteristicsMatrix3(1,1) = 6; 
	
	Matrix<4,4> characteristicsMatrix4;
	characteristicsMatrix4(0,0) = 1; characteristicsMatrix4(0,1) = 3; characteristicsMatrix4(0,2) = 2; characteristicsMatrix4(0,3) = 6;
	characteristicsMatrix4(1,0) = 2; characteristicsMatrix4(1,1) = 4; characteristicsMatrix4(1,2) = 4; characteristicsMatrix4(1,3) = 3;
	characteristicsMatrix4(2,0) = 3; characteristicsMatrix4(2,1) = 5; characteristicsMatrix4(2,2) = 6; characteristicsMatrix4(2,3) = 1;
	characteristicsMatrix4(3,0) = 7; characteristicsMatrix4(3,1) = 9; characteristicsMatrix4(3,2) = 1; characteristicsMatrix4(3,3) = 4;

	Matrix<4,4> characteristicsMatrix5;
	characteristicsMatrix5(0,0) = 3; characteristicsMatrix5(0,1) = 7; characteristicsMatrix5(0,2) = 3; characteristicsMatrix5(0,3) = 0;
	characteristicsMatrix5(1,0) = 0; characteristicsMatrix5(1,1) = 2; characteristicsMatrix5(1,2) = -1; characteristicsMatrix5(1,3) = 1;
	characteristicsMatrix5(2,0) = 5; characteristicsMatrix5(2,1) = 4; characteristicsMatrix5(2,2) = 3; characteristicsMatrix5(2,3) = 2;
	characteristicsMatrix5(3,0) = 6; characteristicsMatrix5(3,1) = 6; characteristicsMatrix5(3,2) = 4; characteristicsMatrix5(3,3) = -1;
	    
	
	
	testNo++;
	std::cout << "Test #" << testNo << ": rank of a matrix" << std::endl;

	if (characteristicsMatrix1.rank() != 3){
	    std::cout << "  Failure: Matrix 1 rank not correct" << std::endl;
	    error++;
	}
	
	if (characteristicsMatrix2.rank() != 2){
	    std::cout << "  Failure: Matrix 2 rank not correct" << std::endl;
	    error++;
	}

	if (characteristicsMatrix3.rank() != 2){
	    std::cout << "  Failure: Matrix 3 rank not correct" << std::endl;
	    error++;
	}
	
	if (characteristicsMatrix4.rank() != 4){
	    std::cout << "  Failure: Matrix 4 rank not correct" << std::endl;
	    error++;
	}
	
	if (characteristicsMatrix5.rank() != 4){
	    std::cout << "  Failure: Matrix 4 rank not correct" << std::endl;
	    error++;
	}
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	
	testNo++;
	std::cout << "Test #" << testNo << ": det of a matrix" << std::endl;

	double det = characteristicsMatrix1.det();
	if (det != 1){
	    std::cout << "  Failure: Matrix 1 det not correct should be: 1 is: "<< det << std::endl;
	    error++;
	}
	
	det = characteristicsMatrix2.det();
	if (det != 0){
	    std::cout << "  Failure: Matrix 2 det not correct should be: 0 is: "<< det << std::endl;
	    error++;
	}
	
	
	det = characteristicsMatrix3.det();
	if (characteristicsMatrix3.det() != -8){
	    std::cout << "  Failure: Matrix 3 det not correct should be: -8 is: "<< det << std::endl;
	    error++;
	}
	
	det = characteristicsMatrix4.det();
	if (det != -26){
	    std::cout << "  Failure: Matrix 4 det not correct should be: -26 is: "<< det << std::endl;
	    error++;
	}
	
	det = characteristicsMatrix5.det();
	if (det != 105){
	    std::cout << "  Failure: Matrix 5 det not correct should be: 105 is: "<< det << std::endl;
	    error++;
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
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
