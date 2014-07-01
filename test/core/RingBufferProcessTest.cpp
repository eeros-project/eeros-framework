#include <iostream>
#include <ostream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/core/SharedMemory.hpp>
#include <eeros/core/RingBuffer.hpp>

#define MEM_SIZE 32

namespace eeros {
	namespace test {
		namespace ringBuffer {
		  
			class Reader : public Runnable {
			public:
				Reader(void* memory, uint32_t size) : rb(memory, size) {
					std::cout << std::hex;
					std::cout << "configure memory for reader: addr = " << memory << std::endl;
					std::cout << std::dec;
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
					std::cout << std::hex;
					std::cout << "configure memory for writer: addr = " << memory << std::endl;
					std::cout << std::dec;
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
	
	int forkPID;
	
	std::cout << "RingBufferProcess test started..." << std::endl;
	
	forkPID = fork();
	switch (forkPID) {
	  case -1:
		std::cout << "fork failed" << std::endl;
		exit(EXIT_FAILURE);
		break;
		
	  case 0: {	// child process
		std::cout << "Creating writer process, pid=" << getpid() << std::endl;
		std::cout << "Allocating memory in shared segment (" << MEM_SIZE << " bytes)..." << std::endl;
		SharedMemory* shm = new SharedMemory("/eeros.shm", MEM_SIZE);
		if (shm->getMemoryPointer() == (void*)kShmError) {
		  std::cout << "Could not get shared memory" << std::endl;
		  exit(EXIT_FAILURE);
		}
		Writer w(shm->getMemoryPointer(), MEM_SIZE);
 		for (int i = 0; i < 5; i++) {
 			w.run();
			usleep(10000);
		}
		std::cout << "terminating writer process, pid=" << getpid() << std::endl;
		exit(EXIT_SUCCESS);
		break;
	  }
	  default:	// parent process
		sleep(1);
		std::cout << "Creating reader process, pid=" << getpid() << std::endl;
		std::cout << "Allocating memory in shared segment (" << MEM_SIZE << " bytes)..." << std::endl;
		SharedMemory* shm = new SharedMemory("/eeros.shm", MEM_SIZE);
		if (shm->getMemoryPointer() == (void*)kShmError) {
		  std::cout << "Could not get shared memory" << std::endl;
		  exit(EXIT_FAILURE);
		}
		Reader r(shm->getMemoryPointer(), MEM_SIZE);
  		for (int i = 0; i < 7; i++) {
			r.run();
			usleep(10000);
  		}
		wait(NULL);
		std::cout << "terminating reader process, pid=" << getpid() << std::endl;
		exit(EXIT_SUCCESS);
	}
}
