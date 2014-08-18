#include <eeros/control/Saturation.hpp>

#include <iostream>
#include <random>
#include <climits>
#include <ctime>
#include <functional>
#include <type_traits>
#include <array>

#include "../../Utils.hpp"

template <typename C, typename T, unsigned int N>
class SaturationBlockTest {
	public:
		SaturationBlockTest() : randomIndex(0) {
			std::default_random_engine generator;
			generator.seed(std::time(0));
			std::uniform_int_distribution<int> distribution(-1000000, 1000000);
			auto rand = std::bind(distribution, generator);
			for(unsigned int i = 0; i < randomBufferLength; i++) {
				random[i] = rand();
				if(std::is_floating_point<T>::value) random[i] /= 1000.0;
			}
			
			uut.getIn().connect(data);
			uut.enable();
		}
		
		int run() {
			int error = 0;
			
			for(unsigned int t = 0; t < nofTests; t++) {
				C value, lowerLimit, upperLimit, ref;
				for(unsigned int i = 0; i < N; i++) {
					value[i] = getRandom();
					lowerLimit[i] = -getRandom(true);
					upperLimit[i] = getRandom(true);
					ref[i] = value[i]; if(value[i] > upperLimit[i]) ref[i] = upperLimit[i]; if(value[i] < lowerLimit[i]) ref[i] = lowerLimit[i];
				}
				
				data.getSignal().setValue(value);
				uut.setLimit(lowerLimit, upperLimit);
				
				uut.run();
				
				C res = uut.getOut().getSignal().getValue();
				
				for(unsigned int i = 0; i < N; i++) {
					if(!Utils::compareApprox(ref[i], res[i], 0.001)) {
						error++;
						std::cout << "test " << t << ", element " << i << ": Failure (" << value[i] << " -> " << res[i] << " but should be " << ref[i] << " [" << lowerLimit[i] << '/' << upperLimit[i] << ']' << ")" << std::endl;
					}
					else {
						std::cout << "test " << t << ", element " << i << ": OK (" << value[i] << " -> " << res[i] << " [" << lowerLimit[i] << '/' << upperLimit[i] << ']' << ")" << std::endl;
					}
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
		static constexpr unsigned int randomBufferLength = nofTests * 3 * N;
		
		T random[randomBufferLength];
		unsigned int randomIndex;
		
		eeros::control::Output<C> data;
		eeros::control::Saturation<C> uut;
};



int main(int argc, char* argv[]) {
	
	using namespace eeros::math;
	SaturationBlockTest<Matrix<3, 1, int>,    int,    3 * 1> m3x1iTester;
	SaturationBlockTest<Matrix<1, 2, long>,   long,   1 * 2> m1x2lTester;
	SaturationBlockTest<Matrix<2, 2, float>,  float,  2 * 2> m2x2fTester;
	SaturationBlockTest<Matrix<3, 2, double>, double, 3 * 2> m3x2dTester;
	
	SaturationBlockTest<std::array<int,    7>, int,    7> a7iTester;
	SaturationBlockTest<std::array<long,   3>, long,   3> a3lTester;
	SaturationBlockTest<std::array<float,  5>, float,  5> a5fTester;
	SaturationBlockTest<std::array<double, 4>, double, 4> a4dTester;
	
	if(argc == 5 && *(argv[1]) == 'm') { // container: eeros matrix
		unsigned int m = atoi(argv[2]); // number of rows
		unsigned int n = atoi(argv[3]); // number of cols
		char t = *(argv[4]); // element type
		if(m == 3 && n == 1 && t == 'i') return m3x1iTester.run();
		else if(m == 1 && n == 2 && t == 'l') return m1x2lTester.run();
		else if(m == 2 && n == 2 && t == 'f') return m2x2fTester.run();
		else if(m == 3 && n == 2 && t == 'd') return m3x2dTester.run();
		else {
			std::cout << "illegal matrix dimension or type (" << m << 'x' << n << '/' << t << "), only 3x1/i, 1x2/l, 2x2/f and 3x2/d are available." << std::endl;
		}
	}
	else if(argc == 4 && *(argv[1]) == 'a') {
		unsigned int n = atoi(argv[2]); // number of elements
		char t = *(argv[3]); // element type
		if(n == 7 && t == 'i') return a7iTester.run();
		else if(n == 3 && t == 'l') return a7iTester.run();
		else if(n == 5 && t == 'f') return a7iTester.run();
		else if(n == 4 && t == 'd') return a7iTester.run();
		else {
			std::cout << "illegal array dimension or type (" << n << '/' << t << "), only 7/i, 3/l, 5/f and 4/d are available." << std::endl;
		}
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -1;
}
