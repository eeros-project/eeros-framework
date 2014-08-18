#include <eeros/core/System.hpp>
#include <eeros/core/EEROSException.hpp>
#include <windows.h>

using namespace eeros;

double System::getClockResolution() {
	return 1; // TODO
}

double System::getTime() {
	double time;
	static bool initialized = false;
	static LARGE_INTEGER offset;
	static double frequencyToNanoseconds;
	if(!initialized) {
        LARGE_INTEGER performanceFrequency;
        initialized = true;
        QueryPerformanceFrequency(&performanceFrequency);
        QueryPerformanceCounter(&offset);
        frequencyToNanoseconds = (double)performanceFrequency.QuadPart / 1000000000.0;
	}
	LARGE_INTEGER t;
	QueryPerformanceCounter(&t);
	t.QuadPart -= offset.QuadPart;
    double nanoseconds = (double)t.QuadPart / frequencyToNanoseconds;
	time = nanoseconds / 1000000000.0;
	return time;
}

uint64_t System::getTimeNs() {
	return 0; // TODO
}
