#include <config.hpp>
#include <eeros/core/System.hpp>

#if defined(WINDOWS)
#include <windows.h>
#endif

#if defined(POSIX)
#include <time.h>
#endif

#define NS_PER_SEC 1000000000

uint64_t System::timeoffset = 0; // [s]

System::System() {
	// TODO Auto-generated constructor stub

}

System::~System() {
	// TODO Auto-generated destructor stub
}

double System::getTime() {
	double time;
#if defined(POSIX)
	timespec tval;
	clock_gettime(CLOCK_REALTIME, &tval); // TODO use clock_getres()...
	if(System::timeoffset == 0) {
		timeoffset = tval.tv_sec;
	}
	time = tval.tv_sec - timeoffset;
	time += tval.tv_nsec / 1000000000.0;
#endif 
#if defined(WINDOWS)
	static bool initialized = false;
	static LARGE_INTEGER offset;
	static double frequencyToNanoseconds;
	if (!initialized) {
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
#endif
	return time;
}

uint64_t System::getTimeNs() {
	uint64_t time;
#if defined(POSIX)
	timespec tval;
	clock_gettime(CLOCK_REALTIME, &tval); // TODO use clock_getres()...
	if(System::timeoffset == 0) {
		timeoffset = tval.tv_sec;
	}
	time = (tval.tv_sec - timeoffset) * NS_PER_SEC;
	time += tval.tv_nsec;
#endif 
#if defined(WINDOWS)
	// TODO
#endif
	return time;
}
