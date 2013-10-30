#include <iostream>
#include <ostream>
#include <stdlib.h>
#include <unistd.h>

#include <eeros/core/RingBuffer.hpp>
#include <eeros/core/Runnable.hpp>
#include <eeros/core/Executor.hpp>

#define MEM_SIZE 12
#define TIMETOWAIT 15

namespace eeros {
	namespace test {
		namespace ringBuffer {

			class Reader : public Runnable {
			public:
				Reader(void* memory, uint32_t size) : rb(memory, size) {
					// nothing to do;
				}
				
				void run() {
					while(rb.availableToRead()) {
						int x;
						rb.read(&x, 4);
						std::cout << "R: " << x << std::endl;
					}
				}

			private:
				RingBuffer rb;
			};

			class Writer : public Runnable {
			public:
				Writer(void* memory, uint32_t size) : rb(memory, size), counter(1000) {
					// nothing to do;
				}
				
				void run() {
					rb.write(&counter, 4);
					std::cout << "W: " << (int)counter << std::endl;
					counter++;
				}

			private:
				RingBuffer rb;
				int counter;
			};

		};
	};
};
			
int main() {
	using namespace eeros;
	using namespace eeros::test::ringBuffer;
	
	std::cout << "RingBuffer test started..." << std::endl;
	
	std::cout << "Allocating memory (" << MEM_SIZE << " bytes)..." << std::endl;
	void* memory = malloc(MEM_SIZE);
	
	std::cout << "Creating executors..." << std::endl;
	Executor e1(0.1); // 100 ms period time
	Executor e2(1); // 1s period time
	
	std::cout << "Creating reader and add them to executor 1..." << std::endl;
	Reader r(memory, MEM_SIZE);
	e1.addRunnable(r);
	
	std::cout << "Creating writer and add them to executor 2..." << std::endl;
	Writer w(memory, MEM_SIZE);
	e2.addRunnable(w);
	
	std::cout << "Starting executors..." << std::endl;
	e1.start();
	e2.start();
	
	std::cout << "Waiting for " << TIMETOWAIT << " seconds while executors are running" << std::endl;
	sleep(TIMETOWAIT);
	
	std::cout << "Stopping executors..." << std::endl;
	e1.stop();
	e2.stop();
	
	std::cout << "Waiting for executors to terminate..." << std::endl;
	while(!e1.isTerminated() && !e2.isTerminated());
	
	std::cout << "Freeing memory (" << MEM_SIZE << " bytes)..." << std::endl;
	free(memory);
	
	std::cout << "Test done..." << std::endl;
}
