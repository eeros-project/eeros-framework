#include <eeros/control/DeMux.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;

// Test initial values for NaN
TEST(controlDeMuxTest, initialValue) {
	DeMux<2> dm;
	dm.setName("dm");
	
	EXPECT_TRUE(std::isnan(dm.getOut(0).getSignal().getValue()));
	EXPECT_TRUE(std::isnan(dm.getOut(1).getSignal().getValue()));
	try {
		dm.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'dm'"));
	}
}

// #include <eeros/core/Runnable.hpp>
// #include <eeros/math/Matrix.hpp>
// #include <eeros/control/DeMux.hpp>
// #include <iostream>
// #include <fstream>
// #include "../Utils.hpp"
// 
// template <int N, int M, typename T = double>
// class DeMuxBlockTest {
// 	public:
// 		DeMuxBlockTest() {
// 			uut.getIn().connect(data);
// 		}
// 		
// 		int run(const char* filepath) {
// 			std::ifstream file(filepath);
// 			if(!file.is_open()) return -2;
// 			
// 			int line = 0;
// 			int error = 0;
// 			eeros::math::Matrix<N, M, T> testValues;
// 			
// 			while(!file.eof()) {
// 				line++;
// 				for(int m = 0; m < M; m++) {
// 					for(int n = 0; n < N; n++) {
// 						double in;
// 						file >> in; // read input data
// 						testValues(n, m) = in;
// 					}
//  				}
// 				if(file.eof()) break;
// 				
// 				double desiredOut[N * M];
// 				for(int i = 0; i < N * M; i++) {
// 					file >> desiredOut[i]; // read input data
// 				}
// 				if(file.eof()) break;
// 				
// 				data.getSignal().setValue(testValues);
// 				uut.run();
// 
// 				for(int i = 0; i < N * M; i++) {
// 					if(!Utils::compareApprox(desiredOut[i], uut.getOut(i).getSignal().getValue(), 0.001)) {
// 						error++;
// 						std::cout << "line " << line << " expecting " << desiredOut[i] << " calculated " << uut.getOut(i).getSignal().getValue() << std::endl;
// 					}
// 				}
// 			}
// 			file.close();
// 			return error;
// 		}
// 		
// 	protected:
// 		eeros::control::Output<eeros::math::Matrix<N, M, T>> data;
// 		eeros::control::DeMux<N*M, T, eeros::math::Matrix<N, M, T>> uut;
// };
// 
// void illegalArgument(int n, int m) {
// 	std::cout << "Illegal argument (" << n << "x" << m << "), available tests: 1x1, 3x1, 4x1, 2x2, 5x5." << std::endl;
// }
// 
// int main(int argc, char* argv[]) {
// 	
// 	DeMuxBlockTest<1, 1, double> tester1x1;
// 	DeMuxBlockTest<3, 1, double> tester3x1;
// 	DeMuxBlockTest<4, 1, double> tester4x1;
// 	DeMuxBlockTest<2, 2, double> tester2x2;
// 	DeMuxBlockTest<5, 5, double> tester5x5;
// 	
// 	if(argc == 4) {
// 		switch(atoi(argv[1])) {
// 			case 1:
// 				if(atoi(argv[2]) == 1) {
// 					return tester1x1.run(argv[3]);
// 				}
// 				else {
// 					illegalArgument(atoi(argv[1]), atoi(argv[2]));
// 					return -4;
// 				}
// 			case 2:
// 				if(atoi(argv[2]) == 2) {
// 					return tester2x2.run(argv[3]);
// 				}
// 				else {
// 					illegalArgument(atoi(argv[1]), atoi(argv[2]));
// 					return -4;
// 				}
// 			case 3:
// 				if(atoi(argv[2]) == 1) {
// 					return tester3x1.run(argv[3]);
// 				}
// 				else {
// 					illegalArgument(atoi(argv[1]), atoi(argv[2]));
// 					return -4;
// 				}
// 			case 4:
// 				if(atoi(argv[2]) == 1) {
// 					return tester4x1.run(argv[3]);
// 				}
// 				else {
// 					illegalArgument(atoi(argv[1]), atoi(argv[2]));
// 					return -4;
// 				}
// 			case 5:
// 				if(atoi(argv[2]) == 5) {
// 					return tester5x5.run(argv[3]);
// 				}
// 				else {
// 					illegalArgument(atoi(argv[1]), atoi(argv[2]));
// 					return -4;
// 				}
// 			default:
// 				illegalArgument(atoi(argv[1]), atoi(argv[2]));
// 				return -4;
// 		}
// 	}
// 	else {
// 		std::cout << "Illegal number of arguments!" << std::endl;
// 	}
// 	return -3;
// }
