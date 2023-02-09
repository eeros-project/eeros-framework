#include <eeros/core/System.hpp>
#include <eeros/core/EEROSException.hpp>
#include <windows.h>
#include <chrono>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

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
  #ifdef USE_ROS2
  if (rosTimeIsUsed) {
    return rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
  }
  #endif

  auto nsecs = std::chrono::high_resolution_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(nsecs).count();
}
