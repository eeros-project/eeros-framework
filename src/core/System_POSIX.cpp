#include <eeros/core/System.hpp>
#include <eeros/core/Fault.hpp>
#include <time.h>

#define NS_PER_SEC 1000000000
#define CLOCK CLOCK_MONOTONIC_RAW


#define USE_ROS_TIME	//TODO where should this be defined?
#ifdef USE_ROS_TIME
#include <ros/ros.h>
#endif

using namespace eeros;

uint64_t timespec2nsec(struct timespec ts) {
	return static_cast<uint64_t>(ts.tv_sec) * NS_PER_SEC + static_cast<uint64_t>(ts.tv_nsec);
}

double timespec2sec(struct timespec ts) {
	return static_cast<double>(timespec2nsec(ts)) / NS_PER_SEC;
}

double System::getClockResolution() {
	struct timespec ts;
	if(clock_getres(CLOCK, &ts) != 0) {
		throw Fault("Failed to get clock resolution!");
	}
	return timespec2sec(ts);
}

double System::getTime() {
	return static_cast<double>(System::getTimeNs()) / NS_PER_SEC;
}

uint64_t System::getTimeNs() {
#ifndef USE_ROS_TIME
	uint64_t time;
	struct timespec ts;
	if(clock_gettime(CLOCK, &ts) != 0) {
		throw Fault("Failed to get time!");
	}
	return timespec2nsec(ts);
#else 
	auto time = ros::Time::now();
	return time.toNSec();
// 	time.
#endif
}


