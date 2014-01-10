#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>

#include <eeros/core/Executor.hpp>
#include <eeros/core/SharedMemory.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/BlockOutput.hpp>
#include <eeros/control/GlobalSignalProvider.hpp>
#include <eeros/control/SignalBufferReader.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#define TIMETOWAIT 30

namespace eeros {
	namespace test {
		namespace martin {
			
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

int main() {
	using namespace eeros;
	using namespace eeros::control;
	using namespace eeros::logger;
	using namespace eeros::test::martin;
	
	Logger<LogWriter> l('M');
	StreamLogWriter w(std::cout);
	l.set(w);
	
	l.info() << "Martin Test 3 started..." << endl;
	
	l.info() << "Creating executors..." << endl;
	Executor e1(0.01); // 10 ms period time
	Executor e2(0.1); // 100 ms period time
	
	l.info() << "Creating and connecting control system elements..." << endl;
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
	
	{
		auto e = l.info();
		e << "Available signals:" << endl;
		for(std::list<Signal*>::iterator i = Signal::getSignalList()->begin(); i != Signal::getSignalList()->end(); i++) {
			uint32_t length = (*i)->getDimension();
			for(uint32_t j = 0; j < length; j++) {
				e << "  " << (*i)->getLabel(j) << endl;
			}
		}
	}
	
	e1.addRunnable(step1);
	e1.addRunnable(step2);
	e1.addRunnable(sum);
	e1.addRunnable(gain);
	e1.addRunnable(globalSignalProvider);
	
	l.info() << "Creating reader..." << endl;
	Reader r(globalSignalProvider.getSharedMemory(), kSharedMemorySize);
	e2.addRunnable(r);
	
	l.info() << "Starting executors..." << endl;
	e1.start();
	e2.start();
	
	l.info() << "Waiting for " << TIMETOWAIT << " seconds while executors are running" << endl;
	sleep(TIMETOWAIT);

	l.info() << "Stopping executors..." << endl;
	e1.stop();
	e2.stop();
	
	l.info() << "Waiting for executors to terminate..." << endl;
	while(!e1.isTerminated() && !e2.isTerminated());
//	while(!e1.isTerminated());
	
	l.info() << "Test 3 done..." << endl;
}
