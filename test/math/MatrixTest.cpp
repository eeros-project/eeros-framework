#include <cstdlib>
#include <iostream>
#include <cmath>
#include <array>


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

 template < uint8_t N, uint8_t M = 1 > struct uuT {
	  Matrix<N,M> matrix;
	  double det;
	  uint32_t rank;
	  double trace;
	  uint8_t orthogonaly;
	  uint8_t invertible;
	  uint8_t symetric;
	  uint8_t lowerTriangular;
	  uint8_t upperTriangular;
	};

	struct rotTesting{
	  Matrix<3,3> uuT;
	  Matrix<3,3> result;
	  
	};

const double MAX_DEVIATION = 0.1; //in %	
constexpr int NUMBER_OF_ROT_TESTING_DATA = 7; 
	
	
double abs(double a){
  if(a>=0){
    return a;
  }else{
    return -a;
  }
  
}
	
int testZero(){
  int error = 0;;
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
  return error;
  
}
	
int testElementalAccess(){
  int error = 0;;
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
  return error;
  
}
		
int testEye(){
  int error = 0;
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
  return error;
}
	
int testRot(std::array<rotTesting,NUMBER_OF_ROT_TESTING_DATA> testingData){
    int error = 0;
    for(uint32_t i = 0; i < testingData.size();i++){
	  for(uint32_t n = 0; n < testingData[i].uuT.getNrOfRows();n++){
	    for(uint32_t m = 0; m < testingData[i].uuT.getNrOfColums();m++){
	       double maxDeviation = 0;
	       if (testingData[i].result(n,m) == 0){
		maxDeviation = 10e-15; 
	       }else{
		maxDeviation = abs(testingData[i].result(n,m)*MAX_DEVIATION/100);
	       }
	       
		if (testingData[i].uuT(n,m) < (testingData[i].result(n,m) -  maxDeviation) || 
		  testingData[i].uuT(n,m) > (testingData[i].result(n,m) +  maxDeviation) ){
		  std::cout << "  Failure: rotx " << i << n << m << " not like solution" << std::endl;
		error++;
	      }

	    }
	    
	  }
	}
  return error;
}
	
int testRotX(){
  std::array<rotTesting,NUMBER_OF_ROT_TESTING_DATA> rotData;
  rotData[0].uuT.rotx(0);
  rotData[1].uuT.rotx(M_PI/4);
  rotData[2].uuT.rotx(M_PI/2);
  rotData[3].uuT.rotx(M_PI);
  rotData[4].uuT.rotx(2*M_PI);
  rotData[5].uuT.rotx(3*M_PI);
  rotData[6].uuT.rotx(-0.4);
  
  rotData[0].result.eye();
  
  rotData[1].result(0,0) = 1; rotData[1].result(0,1) = 0; rotData[1].result(0,2) = 0;
  rotData[1].result(1,0) = 0; rotData[1].result(1,1) = 0.707106781186548; rotData[1].result(1,2) = -0.707106781186547;
  rotData[1].result(2,0) = 0; rotData[1].result(2,1) = 0.707106781186547; rotData[1].result(2,2) = 0.707106781186548;
  
  rotData[2].result(0,0) = 1; rotData[2].result(0,1) = 0; rotData[2].result(0,2) = 0;
  rotData[2].result(1,0) = 0; rotData[2].result(1,1) = 0; rotData[2].result(1,2) = -1;
  rotData[2].result(2,0) = 0; rotData[2].result(2,1) = 1; rotData[2].result(2,2) = 0;
  
  rotData[3].result(0,0) = 1; rotData[3].result(0,1) = 0; rotData[3].result(0,2) = 0;
  rotData[3].result(1,0) = 0; rotData[3].result(1,1) = -1; rotData[3].result(1,2) = 0;
  rotData[3].result(2,0) = 0; rotData[3].result(2,1) = 0; rotData[3].result(2,2) = -1;
  
  rotData[4].result.eye();
  
  rotData[5].result = rotData[3].result;
  
  rotData[6].result(0,0) = 1; rotData[6].result(0,1) = 0; rotData[6].result(0,2) = 0;
  rotData[6].result(1,0) = 0; rotData[6].result(1,1) = 0.921060994002885; rotData[6].result(1,2) = 0.389418342308651;
  rotData[6].result(2,0) = 0; rotData[6].result(2,1) = -0.389418342308651; rotData[6].result(2,2) = 0.921060994002885;
  
  return testRot(rotData);
  
}
	
int testRotY(){
  std::array<rotTesting,NUMBER_OF_ROT_TESTING_DATA> rotData;
  rotData[0].uuT.roty(0);
  rotData[1].uuT.roty(M_PI/4);
  rotData[2].uuT.roty(M_PI/2);
  rotData[3].uuT.roty(M_PI);
  rotData[4].uuT.roty(2*M_PI);
  rotData[5].uuT.roty(3*M_PI);
  rotData[6].uuT.roty(-0.4);

  
  rotData[0].result.eye();
  
  rotData[1].result(0,0) = 0.707106781186548 ; rotData[1].result(0,1) = 0; rotData[1].result(0,2) = 0.707106781186547;
  rotData[1].result(1,0) = 0; rotData[1].result(1,1) = 1; rotData[1].result(1,2) = 0;
  rotData[1].result(2,0) = -0.707106781186547; rotData[1].result(2,1) = 0; rotData[1].result(2,2) = 0.707106781186548;

  rotData[2].result(0,0) = 0; rotData[2].result(0,1) = 0; rotData[2].result(0,2) = 1;
  rotData[2].result(1,0) = 0; rotData[2].result(1,1) = 1; rotData[2].result(1,2) = 0;
  rotData[2].result(2,0) = -1; rotData[2].result(2,1) = 0; rotData[2].result(2,2) = 0;
  
  rotData[3].result(0,0) = -1; rotData[3].result(0,1) = 0; rotData[3].result(0,2) = 0;
  rotData[3].result(1,0) = 0; rotData[3].result(1,1) = 1; rotData[3].result(1,2) = 0;
  rotData[3].result(2,0) = 0; rotData[3].result(2,1) = 0; rotData[3].result(2,2) = -1;
  
  rotData[4].result.eye();
  
  rotData[5].result = rotData[3].result;
  
  rotData[6].result(0,0) = 0.921060994002885 ; rotData[6].result(0,1) = 0; rotData[6].result(0,2) = -0.389418342308651;
  rotData[6].result(1,0) = 0; rotData[6].result(1,1) = 1; rotData[6].result(1,2) = 0;
  rotData[6].result(2,0) = 0.389418342308651; rotData[6].result(2,1) = 0; rotData[6].result(2,2) = 0.921060994002885;
  
  return testRot(rotData);
}
	
int testRotZ(){
  std::array<rotTesting,NUMBER_OF_ROT_TESTING_DATA> rotData;
  rotData[0].uuT.rotz(0);
  rotData[1].uuT.rotz(M_PI/4);
  rotData[2].uuT.rotz(M_PI/2);
  rotData[3].uuT.rotz(M_PI);
  rotData[4].uuT.rotz(2*M_PI);
  rotData[5].uuT.rotz(3*M_PI);
  rotData[6].uuT.rotz(-0.4);
  
  rotData[0].result.eye();
  
  rotData[1].result(0,0) = 0.707106781186548 ; rotData[1].result(0,1) = -0.707106781186547 ; rotData[1].result(0,2) = 0;
  rotData[1].result(1,0) = 0.707106781186547; rotData[1].result(1,1) = 0.707106781186548; rotData[1].result(1,2) = 0;
  rotData[1].result(2,0) = 0; rotData[1].result(2,1) = 0; rotData[1].result(2,2) = 1;
  
  rotData[2].result(0,0) = 0; rotData[2].result(0,1) = -1; rotData[2].result(0,2) = 0;
  rotData[2].result(1,0) = 1; rotData[2].result(1,1) = 0; rotData[2].result(1,2) = 0;
  rotData[2].result(2,0) = 0; rotData[2].result(2,1) = 0; rotData[2].result(2,2) = 1;
  
  rotData[3].result(0,0) = -1; rotData[3].result(0,1) = 0; rotData[3].result(0,2) = 0;
  rotData[3].result(1,0) = 0; rotData[3].result(1,1) = -1; rotData[3].result(1,2) = 0;
  rotData[3].result(2,0) = 0; rotData[3].result(2,1) = 0; rotData[3].result(2,2) = 1;                  
                   
  rotData[4].result.eye();
  
  rotData[5].result = rotData[3].result;
  
  rotData[6].result(0,0) = 0.921060994002885 ; rotData[6].result(0,1) = 0.389418342308651; rotData[6].result(0,2) = 0;
  rotData[6].result(1,0) = -0.389418342308651; rotData[6].result(1,1) = 0.921060994002885; rotData[6].result(1,2) = 0;
  rotData[6].result(2,0) = 0; rotData[6].result(2,1) = 0; rotData[6].result(2,2) = 1;
  
  return testRot(rotData);
  
  return 0;
}
	
int testSwapingRows(){
  int error = 0;
  
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
  return error;
  
}

int testGaussSorting(){

  int error = 0;
  
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
  return error;	
  
}

int testGaussRowElimination(){
  int error = 0;
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
  return error;
}

int testSkewSymetricMatrix(){
  int error = 0;
  Matrix<3,1> a;
  a(0) = 1;
  a(1) = 2;
  a(2) = 3;
  Matrix<3,1> b;
  b(0) = -5;
  b(1) = -3;
  b(2) = 0;

  Matrix<3,3> knownResultA;
  knownResultA(0,0) = 0; knownResultA(0,1) = -3; knownResultA(0,2) = 2;
  knownResultA(1,0) = 3; knownResultA(1,1) = 0; knownResultA(1,2) = -1;
  knownResultA(2,0) = -2; knownResultA(2,1) = 1; knownResultA(2,2) = 0;

  Matrix<3,3> knownResultB;
  knownResultB(0,0) = 0; knownResultB(0,1) = 0; knownResultB(0,2) = -3;
  knownResultB(1,0) = 0; knownResultB(1,1) = 0; knownResultB(1,2) = 5;
  knownResultB(2,0) = 3; knownResultB(2,1) = -5; knownResultB(2,2) = 0;

  Matrix<3,3> result = Matrix<3,3>::createSkewSymmetricMatrix(a);
  if( result != knownResultA){
      std::cout << "  Failure: createSkewSymmetricMatrix A not returned correctly"<< std::endl;
      error++;
  }

  result = Matrix<3,3>::createSkewSymmetricMatrix(b);
  if( result != knownResultB){
      std::cout << "  Failure: createSkewSymmetricMatrix B not returned correctly"<< std::endl;
      error++;
  }
  return error; 
}

int testCrossProduct(){
  int error = 0;
  Matrix<3,1> a;
  a(0) = 1;
  a(1) = 2;
  a(2) = 3;
  Matrix<3,1> b;
  b(0) = -5;
  b(1) = -3;
  b(2) = 0;
  
  Matrix<3,1> c;
  c(0) = 0;
  c(1) = -1;
  c(2) = 9;
  
  Matrix<3,1> knownResult1;
  knownResult1(0) = 9; 
  knownResult1(1) = -15;
  knownResult1(2) = 7; 
  
  Matrix<3,1> knownResult2;
  knownResult2(0) = -27; 
  knownResult2(1) = 45;
  knownResult2(2) = 5; 

  Matrix<3,1> result = Matrix<3,1>::crossProduct(a,b);
  if( result != knownResult1){
      std::cout << "  Failure: cross product 1 not returned correctly"<< std::endl;
      error++;
  }
  result = Matrix<3,1>::crossProduct(b,c);
  if( result != knownResult2){
      std::cout << "  Failure: cross product 2 not returned correctly"<< std::endl;
      error++;
  }
  return error;
}

int testMultiplyElementWise(){
  int error = 0;
  Matrix<3,3> a;
  a(0,0) = 1; a(0,1) = 4; a(0,2) = 7;
  a(1,0) = 2; a(1,1) = 5; a(1,2) = 8;
  a(2,0) = 3; a(2,1) = 6; a(2,2) = 9;
  Matrix<3,3> knownResult;
  knownResult(0,0) = 1; knownResult(0,1) = 16; knownResult(0,2) = 49;
  knownResult(1,0) = 4; knownResult(1,1) = 25; knownResult(1,2) = 64;
  knownResult(2,0) = 9; knownResult(2,1) = 36; knownResult(2,2) = 81;

  Matrix<3,3> result;
  result = a.multiplyElementWise(a);

  if( result != knownResult){
    std::cout << "  Failure: cross product 2 not returned correctly"<< std::endl;
    error++;
  }
  return error;
	
  
}

int testMultiplyMatrixAndScalar(){
  int error = 0;
  Matrix<3,3> a;
  a(0,0) = 1; a(0,1) = 4; a(0,2) = 7;
  a(1,0) = 2; a(1,1) = 5; a(1,2) = 8;
  a(2,0) = 3; a(2,1) = 6; a(2,2) = 9;
  Matrix<3,3> knownResult;
  knownResult(0,0) = 2; knownResult(0,1) = 8; knownResult(0,2) = 14;
  knownResult(1,0) = 4; knownResult(1,1) = 10; knownResult(1,2) = 16;
  knownResult(2,0) = 6; knownResult(2,1) = 12; knownResult(2,2) = 18;

  Matrix<3,3> result;
  result = a*2.0;

  if( result != knownResult){
    std::cout << "  Failure: multiplicytion with a scalar failed"<< std::endl;
    error++;
  }
  return error;
}

int testDivideMatrixAndScalar(){
  int error = 0;
  Matrix<3,3> a;
  a(0,0) = 1; a(0,1) = 4; a(0,2) = 7;
  a(1,0) = 2; a(1,1) = 5; a(1,2) = 8;
  a(2,0) = 3; a(2,1) = 6; a(2,2) = 9;
  Matrix<3,3> knownResult;
  knownResult(0,0) = 0.5; knownResult(0,1) = 2; knownResult(0,2) = 3.5;
  knownResult(1,0) = 1; knownResult(1,1) = 2.5; knownResult(1,2) = 4;
  knownResult(2,0) = 1.5; knownResult(2,1) = 3; knownResult(2,2) = 4.5;

  Matrix<3,3> result;
  result = a/2.0;

  if( result != knownResult){
    std::cout << "  Failure: division with a scalar failed"<< std::endl;
    error++;
  }
  return error;
}


int main(int argc, char *argv[]) {
	int error = 0;
	int testNo = 1;

	/********** A) Creating and inizializing **********/
	
	// zero()
	std::cout << "Test #" << testNo++ << ": zero()" << std::endl;
	error = error + testZero();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// Element access
	std::cout << "Test #" << testNo++ << ": Element access" << std::endl;
	error = error + testElementalAccess();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// eye()
	std::cout << "Test #" << testNo++ << ": eye()" << std::endl;
	error = error + testEye();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// rotx()
	std::cout << "Test #" << testNo++ << ": rotx()" << std::endl;
	error = error + testRotX();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// roty()
	std::cout << "Test #" << testNo++ << ": roty()" << std::endl;
	error = error + testRotY();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	// rotz()
	std::cout << "Test #" << testNo++ << ": rotz()" << std::endl;
	error = error + testRotZ();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	
	/********** Functions for helping to calculating the rank of a matrixl **********/
	
	//swaping rows
	std::cout << "Test #" << testNo++ << ": swaping Rows" << std::endl;
	error = error + testSwapingRows();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	//gauss sorting matrix
	std::cout << "Test #" << testNo++ << ": Gauss sorting Matrix" << std::endl;
	error = error + testGaussSorting();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	//gauss row elimination
	std::cout << "Test #" << testNo++ << ": gaus row elimination" << std::endl;
	error = error + testGaussRowElimination();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	
	
	
	/********** Functions for calculating some characteristics of the matrix **********/
	
	std::array<uuT<3,3>,2> characteristics33;
	
	characteristics33[0].matrix(0,0) = 1; characteristics33[0].matrix(0,1) = 0; characteristics33[0].matrix(0,2) = 0;
	characteristics33[0].matrix(1,0) = 0; characteristics33[0].matrix(1,1) = 1; characteristics33[0].matrix(1,2) = 0;
	characteristics33[0].matrix(2,0) = 0; characteristics33[0].matrix(2,1) = 0; characteristics33[0].matrix(2,2) = 1;
	characteristics33[0].det = 1;
	characteristics33[0].rank = 3;
	characteristics33[0].trace = 3;
	characteristics33[0].orthogonaly = true;
	characteristics33[0].invertible = true;
	characteristics33[0].symetric = true;
	characteristics33[0].lowerTriangular = true;
	characteristics33[0].upperTriangular = true;
	
	characteristics33[1].matrix(0,0) = 1; characteristics33[1].matrix(0,1) = 3; characteristics33[1].matrix(0,2) = 2;
	characteristics33[1].matrix(1,0) = 2; characteristics33[1].matrix(1,1) = 4; characteristics33[1].matrix(1,2) = 4;
	characteristics33[1].matrix(2,0) = 3; characteristics33[1].matrix(2,1) = 5; characteristics33[1].matrix(2,2) = 6;
	characteristics33[1].det = 0;
	characteristics33[1].rank = 2;
	characteristics33[1].trace = 11;
	characteristics33[1].orthogonaly = false;
	characteristics33[1].invertible = false;
	characteristics33[1].symetric = false;
	characteristics33[1].lowerTriangular = false;
	characteristics33[1].upperTriangular = false;

	std::array<uuT<2,2>,1> characteristics22;
	
	characteristics22[0].matrix(0,0) = 2; characteristics22[0].matrix(0,1) = 5; 
	characteristics22[0].matrix(1,0) = 4; characteristics22[0].matrix(1,1) = 6; 
	characteristics22[0].det = -8;
	characteristics22[0].rank = 2;
	characteristics22[0].trace = 8;
	characteristics22[0].orthogonaly = false;
	characteristics22[0].invertible = true;
	characteristics22[0].symetric = false;
	characteristics22[0].lowerTriangular = false;
	characteristics22[0].upperTriangular = false;

	std::array<uuT<4,4>,4> characteristics44;
		
	characteristics44[0].matrix(0,0) = 1; characteristics44[0].matrix(0,1) = 3; characteristics44[0].matrix(0,2) = 2; characteristics44[0].matrix(0,3) = 6;
	characteristics44[0].matrix(1,0) = 2; characteristics44[0].matrix(1,1) = 4; characteristics44[0].matrix(1,2) = 4; characteristics44[0].matrix(1,3) = 3;
	characteristics44[0].matrix(2,0) = 3; characteristics44[0].matrix(2,1) = 5; characteristics44[0].matrix(2,2) = 6; characteristics44[0].matrix(2,3) = 1;
	characteristics44[0].matrix(3,0) = 7; characteristics44[0].matrix(3,1) = 9; characteristics44[0].matrix(3,2) = 1; characteristics44[0].matrix(3,3) = 4;
	characteristics44[0].det = -26;
	characteristics44[0].rank = 4;
	characteristics44[0].trace = 15;
	characteristics44[0].orthogonaly = false;
	characteristics44[0].invertible = true;
	characteristics44[0].symetric = false;
	characteristics44[0].lowerTriangular = false;
	characteristics44[0].upperTriangular = false;
		
	characteristics44[1].matrix(0,0) = 3; characteristics44[1].matrix(0,1) = 7; characteristics44[1].matrix(0,2) = 3; characteristics44[1].matrix(0,3) = 0;
	characteristics44[1].matrix(1,0) = 0; characteristics44[1].matrix(1,1) = 2; characteristics44[1].matrix(1,2) = -1; characteristics44[1].matrix(1,3) = 1;
	characteristics44[1].matrix(2,0) = 5; characteristics44[1].matrix(2,1) = 4; characteristics44[1].matrix(2,2) = 3; characteristics44[1].matrix(2,3) = 2;
	characteristics44[1].matrix(3,0) = 6; characteristics44[1].matrix(3,1) = 6; characteristics44[1].matrix(3,2) = 4; characteristics44[1].matrix(3,3) = -1;
	characteristics44[1].det = 105;
	characteristics44[1].rank = 4;
	characteristics44[1].trace = 7;
	characteristics44[1].orthogonaly = false;
	characteristics44[1].invertible = true;
	characteristics44[1].symetric = false;
	characteristics44[1].lowerTriangular = false;
	characteristics44[1].upperTriangular = false;
	
	characteristics44[2].matrix(0,0) = 3; characteristics44[2].matrix(0,1) = 0; characteristics44[2].matrix(0,2) = 0; characteristics44[2].matrix(0,3) = 0;
	characteristics44[2].matrix(1,0) = 1; characteristics44[2].matrix(1,1) = 2; characteristics44[2].matrix(1,2) = 0; characteristics44[2].matrix(1,3) = 0;
	characteristics44[2].matrix(2,0) = 2; characteristics44[2].matrix(2,1) = 3; characteristics44[2].matrix(2,2) = 3; characteristics44[2].matrix(2,3) = 0;
	characteristics44[2].matrix(3,0) = 3; characteristics44[2].matrix(3,1) = 2; characteristics44[2].matrix(3,2) = 4; characteristics44[2].matrix(3,3) = -1;
	characteristics44[2].det = -18;
	characteristics44[2].rank = 4;
	characteristics44[2].trace = 7;
	characteristics44[2].orthogonaly = false;
	characteristics44[2].invertible = true;
	characteristics44[2].symetric = false;
	characteristics44[2].lowerTriangular = true;
	characteristics44[2].upperTriangular = false;
	
	
	characteristics44[3].matrix(0,0) = 3; characteristics44[3].matrix(0,1) = 4.20; characteristics44[3].matrix(0,2) = 1.3; characteristics44[3].matrix(0,3) = 0.340;
	characteristics44[3].matrix(1,0) = 0.0; characteristics44[3].matrix(1,1) = 2; characteristics44[3].matrix(1,2) = 1.3; characteristics44[3].matrix(1,3) = 2;
	characteristics44[3].matrix(2,0) = 0.0; characteristics44[3].matrix(2,1) = 0.0; characteristics44[3].matrix(2,2) = 3.10; characteristics44[3].matrix(2,3) = 1.90;
	characteristics44[3].matrix(3,0) = 0.0; characteristics44[3].matrix(3,1) = 0.0; characteristics44[3].matrix(3,2) = 0.0; characteristics44[3].matrix(3,3) = -0.10;
	characteristics44[3].det = -1.86;
	characteristics44[3].rank = 4;
	characteristics44[3].trace = 8;
	characteristics44[3].orthogonaly = false;
	characteristics44[3].invertible = true;
	characteristics44[3].symetric = false;
	characteristics44[3].lowerTriangular = false;
	characteristics44[3].upperTriangular = true;
	

	std::cout << "Test #" << testNo++ << ": rank of a matrix" << std::endl;
	
	for(uint32_t i = 0; i < characteristics22.size();i++){
	  if (characteristics22[i].matrix.rank() != characteristics22[i].rank){
	    std::cout << "  Failure: Matrix22 " << i << " rank not correct" << std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i <  characteristics33.size();i++){
	  if (characteristics33[i].matrix.rank() != characteristics33[i].rank){
	    std::cout << "  Failure: Matrix33 " << i << " rank not correct" << std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i <  characteristics44.size();i++){
	  if (characteristics44[i].matrix.rank() != characteristics44[i].rank){
	    std::cout << "  Failure: Matrix44 " << i << " rank not correct" << std::endl;
	    error++;
	  }
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
		
	std::cout << "Test #" << testNo++ << ": det of a matrix" << std::endl;

	for(uint32_t i = 0; i < characteristics22.size();i++){
	  double maxDeviation = abs(characteristics22[i].det*MAX_DEVIATION/100);
	  if ( characteristics22[i].matrix.det() < (characteristics22[i].det - maxDeviation ) || 
	      characteristics22[i].matrix.det() > (characteristics22[i].det +  maxDeviation) ){
		std::cout << "  Failure: Matrix22 " << i << " det not correct is: "<< characteristics22[i].matrix.det() << std::endl;
		error++;
	      }
	}
	      
	
	for(uint32_t i = 0; i < characteristics33.size();i++){
	  double maxDeviation = abs(characteristics33[i].det*MAX_DEVIATION/100);
	  if ( characteristics33[i].matrix.det() < (characteristics33[i].det -  maxDeviation) || 
	      characteristics33[i].matrix.det() > (characteristics33[i].det +  maxDeviation) ){
		std::cout << "  Failure: Matrix33 " << i << " det not correct is: "<< characteristics33[i].matrix.det() << std::endl;
		error++;
	      }
	}
	
	for(uint32_t i = 0; i < characteristics44.size();i++){
	   double maxDeviation = abs(characteristics44[i].det*MAX_DEVIATION/100);
	  if ( characteristics44[i].matrix.det() < (characteristics44[i].det -  maxDeviation) || 
	      characteristics44[i].matrix.det() > (characteristics44[i].det +  maxDeviation) ){
		std::cout << "  Failure: Matrix44 " << i << " det not correct is: "<< characteristics44[i].matrix.det() << std::endl;
		error++;
	      }
	}

	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	

	
	
	std::cout << "Test #" << testNo++ << ": trace of a matrix" << std::endl;

	for(uint32_t i = 0; i < characteristics22.size();i++){
	  double maxDeviation = abs(characteristics22[i].trace*MAX_DEVIATION/100);
	  if ( characteristics22[i].matrix.trace() < (characteristics22[i].trace - maxDeviation ) || 
	      characteristics22[i].matrix.trace() > (characteristics22[i].trace +  maxDeviation) ){
		std::cout << "  Failure: Matrix22 " << i << " trace not correct is: "<< characteristics22[i].matrix.trace() << std::endl;
		error++;
	      }
	}
	      
	
	for(uint32_t i = 0; i < characteristics33.size();i++){
	  double maxDeviation = abs(characteristics33[i].trace*MAX_DEVIATION/100);
	  if ( characteristics33[i].matrix.trace() < (characteristics33[i].trace -  maxDeviation) || 
	      characteristics33[i].matrix.trace() > (characteristics33[i].trace +  maxDeviation) ){
		std::cout << "  Failure: Matrix33 " << i << " trace not correct is: "<< characteristics33[i].matrix.trace() << std::endl;
		error++;
	      }
	}
	
	for(uint32_t i = 0; i < characteristics44.size();i++){
	   double maxDeviation = abs(characteristics44[i].trace*MAX_DEVIATION/100);
	  if ( characteristics44[i].matrix.trace() < (characteristics44[i].trace -  maxDeviation) || 
	      characteristics44[i].matrix.trace() > (characteristics44[i].trace +  maxDeviation) ){
		std::cout << "  Failure: Matrix44 " << i << " trace not correct is: "<< characteristics44[i].matrix.trace() << std::endl;
		error++;
	      }
	}

	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	/********** Functions for checking the matrix characteristics **********/
	
	std::cout << "Test #" << testNo++ << ": orthogonaly of a matrix" << std::endl;
	
	for(uint32_t i = 0; i < characteristics22.size();i++){
	  if(characteristics22[i].matrix.isOrthogonal() != characteristics22[i].orthogonaly){
	    std::cout << "  Failure: Matrix22 " << i << " orthogonaly not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics33.size();i++){
	  if(characteristics33[i].matrix.isOrthogonal() != characteristics33[i].orthogonaly){
	    std::cout << "  Failure: Matrix33 " << i << " orthogonaly not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics44.size();i++){
	  if(characteristics44[i].matrix.isOrthogonal() != characteristics44[i].orthogonaly){
	    std::cout << "  Failure: Matrix44 " << i << " orthogonaly not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	

	std::cout << "Test #" << testNo++ << ": if matrix is invertible" << std::endl;
	
	for(uint32_t i = 0; i < characteristics22.size();i++){
	  if(characteristics22[i].matrix.isInvertible() != characteristics22[i].invertible){
	    std::cout << "  Failure: Matrix22 " << i << " invertibility not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics33.size();i++){
	  if(characteristics33[i].matrix.isInvertible() != characteristics33[i].invertible){
	    std::cout << "  Failure: Matrix33 " << i << " invertibility not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics44.size();i++){
	  if(characteristics44[i].matrix.isInvertible() != characteristics44[i].invertible){
	    std::cout << "  Failure: Matrix44 " << i << " invertibility not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	

	std::cout << "Test #" << testNo++ << ": if matrix is symetric" << std::endl;

	for(uint32_t i = 0; i < characteristics22.size();i++){
	  if(characteristics22[i].matrix.isSymmetric() != characteristics22[i].symetric){
	    std::cout << "  Failure: Matrix22 " << i << " symetrie not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics33.size();i++){
	  if(characteristics33[i].matrix.isSymmetric() != characteristics33[i].symetric){
	    std::cout << "  Failure: Matrix33 " << i << " symetrie not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics44.size();i++){
	  if(characteristics44[i].matrix.isSymmetric() != characteristics44[i].symetric){
	    std::cout << "  Failure: Matrix44 " << i << " symetrie not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	
	std::cout << "Test #" << testNo++ << ": if matrix is lower triangular" << std::endl;

	for(uint32_t i = 0; i < characteristics22.size();i++){
	  if(characteristics22[i].matrix.isLowerTriangular() != characteristics22[i].lowerTriangular){
	    std::cout << "  Failure: Matrix22 " << i << " lower triangular not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics33.size();i++){
	  if(characteristics33[i].matrix.isLowerTriangular() != characteristics33[i].lowerTriangular){
	    std::cout << "  Failure: Matrix33 " << i << " lower triangular not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics44.size();i++){
	  if(characteristics44[i].matrix.isLowerTriangular() != characteristics44[i].lowerTriangular){
	    std::cout << "  Failure: Matrix44 " << i << " lower triangular not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;

	std::cout << "Test #" << testNo++ << ": if matrix is upper triangular" << std::endl;

	for(uint32_t i = 0; i < characteristics22.size();i++){
	  if(characteristics22[i].matrix.isUpperTriangular() != characteristics22[i].upperTriangular){
	    std::cout << "  Failure: Matrix22 " << i << " upper triangular not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics33.size();i++){
	  if(characteristics33[i].matrix.isUpperTriangular() != characteristics33[i].upperTriangular){
	    std::cout << "  Failure: Matrix33 " << i << " upper triangular not detected correctly"<< std::endl;
	    error++;
	  }
	}
	
	for(uint32_t i = 0; i < characteristics44.size();i++){
	  if(characteristics44[i].matrix.isUpperTriangular() != characteristics44[i].upperTriangular){
	    std::cout << "  Failure: Matrix44 " << i << " upper triangular not detected correctly"<< std::endl;
	    error++;
	  }
	}
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "Test #" << testNo++ << ": test creat skew symmetric matrix" << std::endl;
	error = error + testSkewSymetricMatrix();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "Test #" << testNo++ << ": test cross product" << std::endl;
	error = error + testCrossProduct();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "Test #" << testNo++ << ": test multiply element wise" << std::endl;
	error = error + testMultiplyElementWise();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "Test #" << testNo++ << ": test multiply matrix and scalar" << std::endl;
	error = error + testMultiplyMatrixAndScalar();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	std::cout << "Test #" << testNo++ << ": test divide matrix and scalar" << std::endl;
	error = error + testDivideMatrixAndScalar();
	std::cout << "  Test finished with " << error << " error(s)" << std::endl;
	
	
	
	if (error == 0)
		std::cout << "matrix test succeeded" << std::endl;
	else
		std::cout << "matrix test failed: " << error << " errors" << std::endl;

	return error;
}
