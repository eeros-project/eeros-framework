/*
 * System.cpp
 *
 *  Created on: 22.04.2013
 *      Author: zueger1
 */

#include <config.hpp>
#include <eeros/core/System.hpp>

#if defined(WINDOWS)
#include <windows.h>
#endif

#if defined(POSIX)
#include <time.h>
#endif

uint64_t System::timeoffset = 0;

System::System()
{
	// TODO Auto-generated constructor stub

}

System::~System()
{
	// TODO Auto-generated destructor stub
}

double System::getTime() {
	double time;
#if defined(POSIX)
	timespec tval;
	clock_gettime(CLOCK_REALTIME, &tval); // TODO use clock_getres()...
	if(System::timeoffset == 0)
	{
		timeoffset = tval.tv_sec;
	}
	time = tval.tv_sec - timeoffset;
	time += tval.tv_nsec / 1000000000.0;
#endif 
#if defined(WINDOWS)
	// TODO
	time = 0;
#endif
	return time;
}
