#include <eeros/core/Runnable.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/DeMux.hpp>
#include <iostream>
#include <fstream>
#include "../../Utils.hpp"

#define nofOperations 4

template <int M, int N, typename T = double>
class ScalarOpsTest {
	public:
		ScalarOpsTest() {
			testMatrix.zero();
			for(int i = 0; i < nofOperations; i++) {
				refRes[i].zero();
				calcRes[i].zero();
			}
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			if(!file.is_open()) return -2;
			
			int line = 0;
			int error = 0;
			
			while(!file.eof()) {
				line++;
				
				file >> scalar; // first value is the scalar
				
				for(int n = 0; n < N; n++) { // next MxN values is the matrix
					for(int m = 0; m < M; m++) {
						double in;
						file >> in;
						testMatrix(m, n) = in;
					}
 				}
				if(file.eof()) break;

				for(int i = 0; i < nofOperations; i++) { // the last values are the reference results
					for(int n = 0; n < N; n++) {
						for(int m = 0; m < M; m++) {
							double in;
							file >> in;
							refRes[i](m, n) = in;
						}
					}
					if(file.eof()) break;
				}
				if(file.eof()) break;
				
				calcRes[0] = testMatrix + scalar;
				calcRes[1] = testMatrix - scalar;
				calcRes[2] = testMatrix * scalar;
				calcRes[3] = testMatrix / scalar;
				
				for(int i = 0; i < nofOperations; i++) {
					for(int x = 0; x < N * M; x++) {
						if(!Utils::compareApprox(refRes[i][x], calcRes[i][x], 0.001)) {
							error++;
							std::cout << "line " << line << " expecting " << refRes[i](x) << " calculated " << calcRes[i](x) << std::endl;
						}
					}
				}
			}
			file.close();
			return error;
		}
		
	private:
		T scalar;
		eeros::math::Matrix<M, N, T> testMatrix;
		eeros::math::Matrix<M, N, T> refRes[nofOperations];
		eeros::math::Matrix<M, N, T> calcRes[nofOperations];
};

void illegalArgument(int n, int m) {
	std::cout << "Illegal argument (" << n << "x" << m << "), available tests: 4x1, 1x4, 2x2, 3x2." << std::endl;
}

int main(int argc, char* argv[]) {
	
	ScalarOpsTest<4, 1, double> tester4x1d;
	ScalarOpsTest<1, 4, double> tester1x4d;
	ScalarOpsTest<2, 2, double> tester2x2d;
	ScalarOpsTest<3, 2, double> tester3x2d;
	
	if(argc == 4) {
		switch(atoi(argv[1])) {
			case 1:
				if(atoi(argv[2]) == 4) {
					return tester1x4d.run(argv[3]);
				}
				else {
					illegalArgument(atoi(argv[1]), atoi(argv[2]));
					return -4;
				}
			case 2:
				if(atoi(argv[2]) == 2) {
					return tester2x2d.run(argv[3]);
				}
				else {
					illegalArgument(atoi(argv[1]), atoi(argv[2]));
					return -4;
				}
			case 3:
				if(atoi(argv[2]) == 2) {
					return tester3x2d.run(argv[3]);
				}
				else {
					illegalArgument(atoi(argv[1]), atoi(argv[2]));
					return -4;
				}
			case 4:
				if(atoi(argv[2]) == 1) {
					return tester4x1d.run(argv[3]);
				}
				else {
					illegalArgument(atoi(argv[1]), atoi(argv[2]));
					return -4;
				}
			default:
				illegalArgument(atoi(argv[1]), atoi(argv[2]));
				return -4;
		}
	}
	else {
		std::cout << "Illegal number of arguments!" << std::endl;
	}
	return -3;
}
