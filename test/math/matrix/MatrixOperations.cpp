#include <cstdlib>
#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <exception>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros::math;
using namespace std;

template < unsigned int N, unsigned int M, unsigned int U, unsigned int V > struct uuT {
	  Matrix<N,M> A;
	  Matrix<U,V> B;
	  Matrix<N,M> Ainv;
	  Matrix<U,V> Binv;
	  Matrix<N,V> AmultB;
	  Matrix<U,M> BmultA;
	  Matrix<N,M> AaddB;
	  Matrix<N,M> AsubB;
	  Matrix<N,M> BsubA;
	  Matrix<N,M> AdiffB;
	};
	

const int MAX_NR_OF_TEST_CASES = 150;
const double MAX_DEVIATION = 0.001; //in %
std::ifstream file;

template < unsigned int N, unsigned int M> int testMatrices(Matrix<N,M> result, Matrix<N,M> uuT){
    int error = 0;
    for(unsigned int m = 0; m < M; m++){
	  for(unsigned int n = 0; n < N; n++){ 
	    double maxDeviation = 0;
	    if(result(n,m) == 0){
	      maxDeviation = 10e-15;
	    }else if(result(n,m) > 0){
	       maxDeviation = result(n,m)*MAX_DEVIATION/100;
	    }else{
	      maxDeviation = -result(n,m)*MAX_DEVIATION/100;
	      
	    }
	    if ( uuT(n,m) < (result(n,m) -  maxDeviation) || 
		uuT(n,m) > (result(n,m) +  maxDeviation) ){
		error++;
	    }
	  } 
	}
  return error;
}



template < unsigned int N, unsigned int M>  void testForMatrix(string s, string codeWord, Matrix<N,M> *target){
    
    if (s == codeWord){
	Matrix<N,M> newMatrix;
	newMatrix.zero();
	for(unsigned int m = 0; m < M; m++){
	  for(unsigned int n = 0; n < N; n++){
	    file >> newMatrix(m,n);
	  } 
	}
	(*target) = newMatrix;
    }
}


template < unsigned int N, unsigned int M,unsigned int U, unsigned int V> int testFile(uuT<N,M,U,V> data[],string path){
    
    int testCaseNr = 0;
    int aNrOfRows = 0;
    int aNrOfColums = 0; 
    int bNrOfRows = 0; 
    int bNrOfColums = 0; 
    int numberOfTestcases = 0;
    int error = 0;
    string s = "";
    
    file.open(path);
    while (!file.eof()) {
      
      file >> s; // read input data
      if (s == "numberOfTestCases"){
	file >> s;
	file >> numberOfTestcases;
	std::cout << "numberOfTestCases = " << numberOfTestcases<< std::endl;
      }
      if (s == "sizeA"){
	file >> s; //= 
	file >> aNrOfRows;
	file >> s; //x 
	file >> aNrOfColums;
	file >> s; //, 
	file >> s; //sizeB
	file >> s; //=
	file >> bNrOfRows;
	file >> s; //x 
	file >> bNrOfColums;
	std::cout << "got A size = " << aNrOfRows << " x " << aNrOfColums << std::endl;
	std::cout << "got B size = " << bNrOfRows << " x " << bNrOfColums << std::endl;
	if(numberOfTestcases == 0 || aNrOfRows != N || aNrOfColums != M || bNrOfRows != U || bNrOfColums != V || numberOfTestcases > MAX_NR_OF_TEST_CASES){
	  numberOfTestcases = 0; 
	  break;
	}
	
      }
      testForMatrix(s,"A=",&data[testCaseNr].A);
      testForMatrix(s,"B=",&data[testCaseNr].B);
      testForMatrix(s,"Ainv=",&data[testCaseNr].Ainv);
      testForMatrix(s,"Binv=",&data[testCaseNr].Binv);
      testForMatrix(s,"AmultB=",&data[testCaseNr].AmultB);
      testForMatrix(s,"BmultA=",&data[testCaseNr].BmultA);
      testForMatrix(s,"AaddB=",&data[testCaseNr].AaddB);
      testForMatrix(s,"AsubB=",&data[testCaseNr].AsubB);
      testForMatrix(s,"BsubA=",&data[testCaseNr].BsubA);
      testForMatrix(s,"AdiffB=",&data[testCaseNr].AdiffB);
      if (s == "end"){
	  testCaseNr++;
      }
    }
    file.close();

    int aInvErrors = 0;
    int bInvErrors = 0;
    int aMultBErrors = 0;
    int bMultAErrors = 0;
    int aAddBErrors = 0;
    int aSubBErrors = 0;
    int bSubAErrors = 0;
    int bDiffAErrors = 0;
    
    
    for(int i = 0; i< numberOfTestcases; i++){
	 
	  if (data[i].A.det() != 0){ //test if inversion is possible
	    Matrix<N,V> result;
	    result.zero();
	    result = !data[i].A;
	    aInvErrors = aInvErrors + testMatrices(result,data[i].Ainv);
	  }else{
	    std::cout << "!A not possible cause of det = 0: testing skiped"<< std::endl;
	  }
      
	  if (data[i].B.det() != 0){ //test if inversion is possible
	    Matrix<N,V> result;
	    result.zero();
	    result = !data[i].B;
	    bInvErrors = bInvErrors + testMatrices(result,data[i].Binv);
	  }else{
	    std::cout << "!B not possible cause of det = 0: testing skiped"<< std::endl;
	  }
      
	   if (M == U){ //multiplication is possible
	    Matrix<N,V> result;
	    result.zero();
	    result = data[i].A * data[i].B;
	    aMultBErrors = aMultBErrors + testMatrices(result,data[i].AmultB);
	  }else{
	    std::cout << "A * B not possible cause of dimension: testing skiped"<< std::endl;
	  }
	  if(V == N){
	    Matrix<U,M> result;
	    result.zero();
	    result = data[i].B * data[i].A;
	    bMultAErrors = bMultAErrors + testMatrices(result,data[i].BmultA); 
	  }else{
	    std::cout << "B*A not possible cause of dimension: testing skiped"<< std::endl;
	  }
	  if(N == U && M == V){
	    Matrix<N,M> result;
	    result.zero();
	    result = data[i].A + data[i].B;
	    aAddBErrors = aAddBErrors + testMatrices(result,data[i].AaddB); 
	    result = data[i].B + data[i].A;
	    aAddBErrors = aAddBErrors + testMatrices(result,data[i].AaddB); 
	    result = data[i].A - data[i].B;
	    aSubBErrors = aSubBErrors + testMatrices(result,data[i].AsubB); 
	    result = data[i].B - data[i].A;
	    bSubAErrors = bSubAErrors + testMatrices(result,data[i].BsubA); 
	  }else{
	    std::cout << "A+-B, B+-A not possible cause of dimension: testing skiped"<< std::endl;
	  }
	  //result = data[i].A / data[i].B;
	  //diff causes exception
	 // bDiffAErrors = bDiffAErrors + testMatrices(result,data[i].AdiffB);
    }
    std::cout << "Number of A invert errors:" << aInvErrors<< std::endl; 
    std::cout << "Number of B invert errors:" << bInvErrors<< std::endl; 
    std::cout << "Number of A mult B errors:" << aMultBErrors<< std::endl; 
    std::cout << "Number of B mult A errors:" << bMultAErrors<< std::endl; 
    std::cout << "Number of A add B errors:" <<aAddBErrors<< std::endl;
    std::cout << "Number of A sub B errors:" <<aSubBErrors<< std::endl;
    std::cout << "Number of B sub A errors:" <<bSubAErrors<< std::endl;
    std::cout << "Number of B diff A errors:" <<bDiffAErrors<< std::endl;
    
    error = error + aMultBErrors + bMultAErrors + aAddBErrors + aSubBErrors + bSubAErrors + bDiffAErrors + aInvErrors + bInvErrors;
 
    return error;
}


int main( int argc,  char *argv[]) {
  int error = 0;
  
  
  if (argc == 6) {
    unsigned int N = atoi(argv[1]);//number of colums A
    unsigned int M = atoi(argv[2]);//number of rows A
    unsigned int U = atoi(argv[3]);//number of colums B
    unsigned int V = atoi(argv[4]);//number of rows B
    
    std::cout << "start testing" << std::endl;
    if(N==2 && M==2 && U ==2 && V == 2){
      uuT<2,2,2,2> data[MAX_NR_OF_TEST_CASES];
      error = testFile(data,argv[5]); 
    }else if (N==3 && M==3 && U ==3 && V == 3){
      uuT<3,3,3,3> data[MAX_NR_OF_TEST_CASES];
      error = testFile(data,argv[5]);  
    }else if (N==4 && M==4 && U ==4 && V == 4){
      uuT<4,4,4,4> data[MAX_NR_OF_TEST_CASES];
      error = testFile(data,argv[5]);  
    }else{
       std::cout << "Type not supported " << std::endl;
      return -2;
      
    }
    std::cout << "Number of errors:" <<error<< std::endl;
    return error;
  }else{
    std::cout << "illegal number of arguments: "<<  argc << std::endl;
    return -1;
  }
  
  
  return error;
}
