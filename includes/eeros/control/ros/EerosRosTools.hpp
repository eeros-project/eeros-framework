#ifndef EEROS_ROS_TOOLS_HPP
#define EEROS_ROS_TOOLS_HPP

#include <ros/ros.h>
#include <std_msgs/Header.h>

#define NS_PER_SEC 1000000000

namespace eeros {
	namespace control {
		namespace rosTools {
      static ros::Time convertToRosTime(uint64_t timestampNs) __attribute__((unused));
			static ros::Time convertToRosTime(uint64_t timestampNs) {
				ros::Time t;
				t.sec = static_cast<double>(timestampNs) / NS_PER_SEC;
				t.nsec = timestampNs % static_cast<uint64_t>(1e9);
				return t;
			}
			
      static void initNode(std::string name) __attribute__((unused));
      static void initNode(std::string name) {
				char* args[] = {NULL};
				int argc = sizeof(args)/sizeof(args[0]) - 1;
				ros::init(argc, args, name);	// init ROS node and name it
			}
		};
	};
};



#endif /* EEROS_ROS_TOOLS_HPP */