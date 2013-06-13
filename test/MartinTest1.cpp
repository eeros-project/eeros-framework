#include <iostream>
#include <ostream>

#include <eeros/core/Executor.hpp>
#include <eeros/core/Runnable.hpp>

#define TIMETOWAIT 3

class Test : public Runnable
{
public:
	Test() { counter = 0; }
	virtual void run() { counter++; }
	int counter;
};


int main()
{
	std::cout << "Martin Test 1 started..." << std::endl;
	Test test;
	Executor e(0.1); // 100 ms period time
	e.addRunnable(&test);
	e.start();
	std::cout << "waiting for " << TIMETOWAIT << " seconds while executor is counting" << std::endl;
#if defined(WINDOWS)
	Sleep(TIMETOWAIT * 1000);
#else
	sleep(TIMETOWAIT);
#endif
	e.stop();
	std::cout << "waiting for executor to terminate..." << std::endl;
	while(!e.isTerminated());
	std::cout << "test = " << test.counter << std::endl;
	std::cout << "Test 1 done..." << std::endl;
}