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
		
		using Vector = Matrix< 1 , 1 , double >; 
		bool first = true;
		
		int run(const char* filepath) {
			
			std::ifstream file(filepath);
			if (!file.is_open())
				return -2;
				
			int line = 0;
			int error = 0;
			
			std::array<Vector, 3> start = {0.15}; std::array<Vector, 3> end = {0.59};
			Vector velMax = 1.0; Vector accMax = 1.0; Vector decMax = 1.0;
			double dt = 0.001;
			constexpr int dimVec = 3001;
			
			std::array<Vector, 3> yout;
			
			ConstantAccTrajectoryGenerator<Vector> pathPlanner(velMax, accMax, decMax, dt);
			
			Matrix<dimVec,3,double> pp_matlab;
			Matrix<dimVec,3,double> pp_eeros;
			
			// Calculate output
			pathPlanner.push(start, end);
			
			while(!file.eof()) {
				line++;
				
				// read input data
				for(int j = 0; j < 3; j++) {
					file >> pp_matlab(line-1,j);
				}
				
				std::array<Vector, 3> temp;
				if(first) {
					temp = pathPlanner.get(0.000);
					first = false;
				}
				else
					temp = pathPlanner.get(0.001);
				
				for(int j = 0; j < 3; j++) {
					pp_eeros(line-1,j) = temp[j](0);
				}
				
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