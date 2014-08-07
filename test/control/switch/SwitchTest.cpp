#include <eeros/control/Switch.hpp>

#include <iostream>
#include <random>
#include <climits>
#include <functional>

#include "../../Utils.hpp"

template <unsigned int N, typename T>
class SwitchBlockTest {
	public:
		SwitchBlockTest() : uut(0) {
			for(int i = 0; i < N; i++) {
				uut.getIn(i).connect(data[i]);
			}
		}
		
		void generateTestData(T seed) {
			std::default_random_engine generator;
			generator.seed(seed);
			std::uniform_int_distribution<T> distribution(INT_MIN, INT_MAX);
			auto rand = std::bind(distribution, generator);
			
			for(int i = 0; i < N; i++) {
				random[i] = rand();
				data[i].getSignal().setValue(random[i]);
			}
		}
		
		int run() {
			int error = 0;
			
			for(unsigned int t = 0; t < nofTests; t++) {
				generateTestData(t);
				for(int i = 0; i < N; i++) {
					uut.switchToInput(i);
					uut.run();
					if(uut.getOut().getSignal().getValue() != random[i]) {
						error++;
						std::cout << "test " << t << ", input " << i << ": expecting " << random[i] << " get " << uut.getOut().getSignal().getValue() << std::endl;
					}
// 					else {
// 						std::cout << "test " << t << ", input " << i << ": OK (" << random[i] << ")" << std::endl;
// 					}
				}
			}
			return error;
		}
		
	protected:
		T random[N];
		eeros::control::Output<T> data[N];
		eeros::control::Switch<N, T> uut;
		
		static const unsigned int nofTests = 100;
};

int main(int argc, char* argv[]) {
	SwitchBlockTest<2, int> tester2;
	SwitchBlockTest<3, int> tester3;
	SwitchBlockTest<10, int> tester10;
	
	if(argc == 2) {
		switch(atoi(argv[1])) {
			case 2:
				return tester2.run();
				break;
			case 3:
				return tester3.run();
				break;
			case 10:
				return tester10.run();
				break;
			default:
				std::cout << "illegal number of inputs (only 2, 3 or 10 are available)." << std::endl;
				break;
		}
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}
