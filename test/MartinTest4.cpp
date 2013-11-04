#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ComediDevice.hpp>
#include <eeros/hal/ComediDac.hpp>
#include <eeros/hal/ComediAdc.hpp>
#include <eeros/hal/ComediFqd.hpp>

#define TIMETOWAIT 5

namespace eeros {
	namespace test {
		namespace martin {
			
			using namespace eeros::hal;
			
			class TestRunner : public Runnable {
			
			public:
				int adcVal;
				int fqdVal;
				
				TestRunner() : 
				hal(HAL::instance()),
				comediDev0("/dev/comedi0"),
				dac("dac", comediDev0, 1, 1),
				adc("adc", comediDev0, 0, 0),
				fqd("fqd", comediDev0, 11, 8, 10, 9),
				counter(0) {
					// nothing to do
				}
				
				void run() {
					adcVal = adc.get();
					fqdVal = fqd.get();
					
					dac.set(counter);
					counter += 0.1;
				}

			private:
				HAL& hal;
				ComediDevice comediDev0;
				ComediDac dac;
				ComediAdc adc;
				ComediFqd fqd;
				double counter;
			};

		};
	};
};

int main() {
	using namespace eeros;
	using namespace eeros::test::martin;
	
	std::cout << "Martin Test 4 started..." << std::endl;
	
	std::cout << "Creating executor..." << std::endl;
	Executor e(0.1); // 100 ms period time
	
	std::cout << "Creating test runner..." << std::endl;
	TestRunner t;
	
	std::cout << "Starting executor..." << std::endl;
	e.addRunnable(t);
	e.start();
	
	std::cout << "Waiting for " << TIMETOWAIT << " seconds while executor is running" << std::endl;
	sleep(TIMETOWAIT);
 
	std::cout << "Stopping executor..." << std::endl;
	e.stop();
	
	std::cout << "Waiting for executor to terminate..." << std::endl;
	while(!e.isTerminated());
	
	std::cout << "ADC value: " << t.adcVal << std::endl;
	
	std::cout << "FQD value: " << t.fqdVal << std::endl;
	
	std::cout << "Martin Test 4 done!" << std::endl;
}
