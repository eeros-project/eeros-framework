#include <eeros/control/Saturation.hpp>

#include <iostream>
#include <random>
#include <climits>
#include <ctime>
#include <functional>
#include <type_traits>

#include "../../Utils.hpp"

template <typename T>
class SaturationBlockTest {
	public:
		SaturationBlockTest() : randomIndex(0) {
			std::default_random_engine generator;
			generator.seed(std::time(0));
			std::uniform_int_distribution<int> distribution(-1000000, 1000000);
			auto rand = std::bind(distribution, generator);
			for(unsigned int i = 0; i < randomBufferLength; i++) {
				random[i] = rand();
				if(!std::is_integral<T>::value) random[i] /= 1000;
			}
			
			uut.getIn().connect(data);
			uut.enable();
		}
		
		int run() {
			int error = 0;
			
			for(unsigned int t = 0; t < nofTests; t++) {
				T value = getRandom();
				T lowerLimit = -getRandom(true);
				T upperLimit = getRandom(true);
				T ref = value; if(value > upperLimit) ref = upperLimit; if(value < lowerLimit) ref = lowerLimit;
				
				data.getSignal().setValue(value);
				uut.setLimit(lowerLimit, upperLimit);
				
				uut.run();
				
				T res = uut.getOut().getSignal().getValue();
				
				if(!Utils::compareApprox(ref, res, 0.001)) {
					error++;
					std::cout << "test " << t << ": Failure (" << value << " -> " << res << " but should be " << ref << " [" << lowerLimit << '/' << upperLimit << ']' << ")" << std::endl;
				}
				else {
					std::cout << "test " << t << ": OK (" << value << " -> " << res << " [" << lowerLimit << '/' << upperLimit << ']' << ")" << std::endl;
				}
			}
			return error;
		}
		
	protected:
		T getRandom(bool noNegativeNumbers = false) {
			T r = random[randomIndex++ % randomBufferLength];
			if(noNegativeNumbers && r < 0) r = -r;
			return r;
		}
		
		static constexpr unsigned int nofTests = 100;
		static constexpr unsigned int randomBufferLength = nofTests * 3;
		
		T random[randomBufferLength];
		unsigned int randomIndex;
		
		eeros::control::Output<T> data;
		eeros::control::Saturation<T> uut;
};



int main(int argc, char* argv[]) {
	
	using namespace eeros::math;
	SaturationBlockTest<int> intTester;
	SaturationBlockTest<double> doubleTester;
// 	SaturationBlockTest<Vector3> vectorTester;
// 	SaturationBlockTest<Matrix<4, 4>> matrixTester;
	
	if(argc == 2) {
		char t = *(argv[1]);
		switch(t) {
			case 'i':
				return intTester.run();
				break;
			case 'd':
				return doubleTester.run();
				break;
// 			case 'v':
// 				return vectorTester.run();
// 				break;
// 			case 'm':
// 				return matrixTester.run();
// 				break;
			default:
				std::cout << "illegal type (" << t << "), only 'd' for double, 'v' for vector or 'm' for matrix are available." << std::endl;
				break;
		}
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}
