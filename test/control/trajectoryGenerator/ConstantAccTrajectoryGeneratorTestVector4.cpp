#include <eeros/control/ConstantAccTrajectoryGenerator.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Runnable.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include "../../Utils.hpp"

using namespace eeros::control;
using namespace eeros::math;

union myUnion {
    double dValue;
    uint64_t iValue;
};

class ConstantAccTrajectoryGeneratorTest {
	public:
		ConstantAccTrajectoryGeneratorTest() { }
		
// 		using Vector = Matrix< 1 , 1 , double >; 
		bool first = true;
		
		int run(const char* filepath) {
			
			std::ifstream file(filepath);
			if (!file.is_open())
				return -2;

			int line = 0;
			int error = 0;
			uint64_t timestamp = 0;
			
			Vector4 startPos; startPos << 4, 1, 2.4, 1.7;
			Vector4 startVel; startVel << 0.0, 0.0, 0.0, 0.0;
			Vector4 startAcc; startAcc << 0.0, 0.0, 0.0, 0.0;
			Vector4 endPos; endPos << 3, 5.6, 4.5, 8;
			Vector4 endVel; endVel << 0.0, 0.0, 0.0, 0.0;
			Vector4 endAcc; endAcc << 0.0, 0.0, 0.0, 0.0;
						
			std::array<Vector4, 3> start = {startPos, startVel, startAcc};
			std::array<Vector4, 3> end = {endPos, endVel, endAcc};
			Vector4 velMax; velMax << 5.0, 5.0, 5.0, 5.0;
			Vector4 accMax; accMax << 3.0, 3.0, 4.0, 4.0;
			Vector4 decMax; decMax << 3.0, 4.0, 3.0, 4.0;
			
			double dt = 0.001;
			constexpr int dimVec = 3001;
			
			std::array<Vector4, 3> yout;
			
			ConstantAccTrajectoryGenerator<Vector4> pathPlanner(velMax, accMax, decMax, dt);
			
			Matrix<dimVec,12,double> pp_matlab;
			Matrix<dimVec,12,double> pp_eeros;
						
			// Calculate output
			pathPlanner.push(start, end);
			
			while(!file.eof()) {
				line++;
				
				// read input data
				for(int j = 0; j < 12; j++) {
					file >> pp_matlab(line-1,j);
				}
				
				std::array<Vector4, 3> temp;
				if(first) {
					temp = pathPlanner.get(0.000);
					first = false;
				}
				else
					temp = pathPlanner.get(0.001);
				
				Vector4 p = temp[0];
				Vector4 v = temp[1];
				Vector4 a = temp[2];
				
				pp_eeros(line-1,0)  = p(0); pp_eeros(line-1,1)  = v(0); pp_eeros(line-1,2)  = a(0);
				pp_eeros(line-1,3)  = p(1); pp_eeros(line-1,4)  = v(1); pp_eeros(line-1,5)  = a(1);
				pp_eeros(line-1,6)  = p(2); pp_eeros(line-1,7)  = v(2); pp_eeros(line-1,8)  = a(2);
				pp_eeros(line-1,9)  = p(3); pp_eeros(line-1,10) = v(3); pp_eeros(line-1,11) = a(3);
				
				if(line == dimVec-1) break;
				
				for(int j = 0; j < 3; j++){
					double res = pp_eeros(line-1,j);
					double ref = pp_matlab(line-1,j);
					if(!Utils::compareApprox(ref, res, 0.001)) {
						error++;
					std::cout << "(i,j): (" << line-1 << ", " << j << ") " << "; expecting  out: " << ref << "; calculated: " << res << std::endl;
					}
				}
			}
			if (error == 0)
				std::cout << "No errors!" << std::endl;

			file.close();
			return error;
	}
};

int main(int argc, char* argv[]) {
	ConstantAccTrajectoryGeneratorTest tester;
	
	if (argc == 2) {
		return tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}