#include <eeros/core/Runnable.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Mux.hpp>
#include <iostream>
#include <fstream>
#include "../../Utils.hpp"

template <int M, int N, typename T = double>
class MuxBlockTest {
	public:
		MuxBlockTest() {
			for(unsigned int i = 0; i < M * N; i++) {
				uut.getIn(i).connect(data[i]);
			}
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			if(!file.is_open()) return -2;
			
			int line = 0;
			int error = 0;
			
			while(!file.eof()) {
				line++;
				
				// Read test values
				T testValues[M * N];
				for(int i = 0; i < M * N; i++) {
					double in;
					file >> in; // read input data
					testValues[i] = in;
 				}
				if(file.eof()) break;
				
				// Read reference results
				double refRes[M * N];
				for(int i = 0; i < M * N; i++) {
					file >> refRes[i]; // read input data
				}
				if(file.eof()) break;
				
				for(unsigned int i = 0; i < M * N; i++) {
					data[i].getSignal().setValue(testValues[i]);
				}
				uut.run();

				for(int i = 0; i < N * M; i++) {
					if(!Utils::compareApprox(refRes[i], uut.getOut().getSignal().getValue()[i], 0.001)) {
						error++;
						std::cout << "line " << line << " expecting " << refRes[i] << " calculated " << uut.getOut().getSignal().getValue()[i] << std::endl;
					}
				}
			}
			file.close();
			return error;
		}
		
	protected:
		eeros::control::Output<T> data[M * M];
		eeros::control::Mux<M * N, T, eeros::math::Matrix<M, N, T>> uut;
};

void illegalArgument(int n, int m) {
	std::cout << "Illegal argument (" << n << "x" << m << "), available tests: 1x1, 3x1, 4x1, 2x2, 5x5." << std::endl;
}

int main(int argc, char* argv[]) {
	
	MuxBlockTest<1, 1, double> tester1x1;
	MuxBlockTest<3, 1, double> tester3x1;
	MuxBlockTest<4, 1, double> tester4x1;
	MuxBlockTest<2, 2, double> tester2x2;
	MuxBlockTest<5, 5, double> tester5x5;
	
	if(argc == 4) {
		switch(atoi(argv[1])) {
			case 1:
				if(atoi(argv[2]) == 1) {
					return tester1x1.run(argv[3]);
				}
				else {
					illegalArgument(atoi(argv[1]), atoi(argv[2]));
					return -4;
				}
			case 2:
				if(atoi(argv[2]) == 2) {
					return tester2x2.run(argv[3]);
				}
				else {
					illegalArgument(atoi(argv[1]), atoi(argv[2]));
					return -4;
				}
			case 3:
				if(atoi(argv[2]) == 1) {
					return tester3x1.run(argv[3]);
				}
				else {
					illegalArgument(atoi(argv[1]), atoi(argv[2]));
					return -4;
				}
			case 4:
				if(atoi(argv[2]) == 1) {
					return tester4x1.run(argv[3]);
				}
				else {
					illegalArgument(atoi(argv[1]), atoi(argv[2]));
					return -4;
				}
			case 5:
				if(atoi(argv[2]) == 5) {
					return tester5x5.run(argv[3]);
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
