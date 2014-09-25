#include <eeros/core/Thread.hpp>
#include <eeros/core/PeriodicThread.hpp>

#include <string>
#include <iostream>
#include <unistd.h>

using namespace eeros;

class ThreadTest : public Thread {
public:
	ThreadTest(std::string n) : name(n) {
		std::cout << "Thread '" << name << "' created with id " << getId() << std::endl;
	}

protected:
	virtual void run() {
		usleep(6000);
		std::cout << "Thread '" << name << "' finished." << std::endl;
	}
	
private:
	std::string name;
};

class PeriodicTest : public PeriodicThread {
public:
	PeriodicTest(std::string n, double period, double delay) : name(n), counter(0), PeriodicThread(period, delay, false) {
		std::cout << "Periodic thread '" << name << "' created with id " << getId() << std::endl;
	}
	
protected:
	virtual void run() {
		counter++;
		std::cout << '.';
		if(counter > 20) {
			stop();
			std::cout << std::endl;
		}
	}
	
private:
	std::string name;
	uint32_t counter;
};


int main(int argc, char* argv[]) {
	ThreadTest a("a");
	PeriodicTest b("b", 0.01, 0.5);
	a.join();
	b.join();
} 
