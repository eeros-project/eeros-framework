#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/core/Executor.hpp>
#include <eeros/core/SharedMemory.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/BlockOutput.hpp>
#include <eeros/control/GlobalSignalProvider.hpp>
#include <eeros/control/SignalBufferReader.hpp>

#define TIMETOWAIT 30

namespace eeros {
	namespace test {
		namespace vinci {

		class Reader : public Runnable {
		public:
			Reader(void* memory, uint32_t size) : r(memory, size) {}
			
			void run() {
				while(r.signalTypeAvailableToRead() == eeros::control::kSignalTypeReal) {
					r.readRealSignal(&id, &ts, &val);
					std::cout << '#' << id << ' ' << ts << ':' << val << std::endl;
				}
			}

		private:
			eeros::control::SignalBufferReader r;
			sigid_t id;
			uint64_t ts;
			double val;
		};

		};
	};
};

static bool run = true;

//Ctrl-C Signal catch
void signal_callback_handler(int signum) {
	   printf("Test will Terminate\nCaught signal %d\n",signum);
	   // Cleanup and close up stuff here
	 run = false;
	   // Terminate program
	   //exit(signum);
}

int main() {
	using namespace eeros;
	using namespace eeros::control;
	using namespace eeros::test::vinci;
	
	// Register signal and signal handler
	signal(SIGINT, signal_callback_handler);
	
	std::cout << "Vinci Test 1 started..." << std::endl;
	
	std::cout << "Creating executors..." << std::endl;
	Executor e1(0.001); // 1 ms period time
	//e2 für den Reader, Writer ist GlobalSignalProvider mit Takt von e1
	Executor e2(0.1); // 100 ms period time
	
	std::cout << "Creating and connecting control system elements..." << std::endl;
	Step step1(0.0, 5.0, 20.0);
	step1.getOut().setName("Step 1");
	
	Step step2(0.0, -8.0, 25.0);
	step2.getOut().setName("Step 2");
	
	Sum sum;
	sum.getOut().setName("Sum");
	sum.negateInput(1);
	
	Gain gain(2.0); // A/N
	gain.getOut().setName("Gain");
	
	GlobalSignalProvider globalSignalProvider;
	
	
	sum.getIn(0).connect(step1.getOut());
	sum.getIn(1).connect(step2.getOut());
	gain.getIn().connect(sum.getOut());
	
	std::cout << "Available signals:" << std::endl;
	for(std::list<Signal*>::iterator i = Signal::getSignalList()->begin(); i != Signal::getSignalList()->end(); i++) {
		uint32_t length = (*i)->getDimension();
		for(uint32_t j = 0; j < length; j++) {
			std::cout << "  " << (*i)->getLabel(j) << std::endl;
		}
	}
	
	e1.addRunnable(step1);
	e1.addRunnable(step2);
	e1.addRunnable(sum);
	e1.addRunnable(gain);
	e1.addRunnable(globalSignalProvider);
	
	std::cout << "Creating reader..." << std::endl;
	Reader r(globalSignalProvider.getSharedMemory(), kSharedMemorySize);
	e2.addRunnable(r);
	
	std::cout << "Starting executors..." << std::endl;
	e1.start();
	e2.start();
	
	//std::cout << "Waiting for " << TIMETOWAIT << " seconds while executors are running" << std::endl;
	std::cout << "Waiting for termination Ctrl+C" << std::endl;
	//sleep(TIMETOWAIT);
	while(run);
 
	std::cout << "Stopping executors..." << std::endl;
	e1.stop();
	e2.stop();
	
	std::cout << "Waiting for executors to terminate..." << std::endl;
	while(!e1.isTerminated() && !e2.isTerminated());
//	while(!e1.isTerminated());
	
	std::cout << "Vinci Test 1 done..." << std::endl;
}
