#ifndef EEROS_ROS_TOOLS_HPP
#define EEROS_ROS_TOOLS_HPP

#include <ros/ros.h>
// #include <ros/time.h>
#include <std_msgs/Header.h>

#define NS_PER_SEC 1000000000

namespace eeros {
	namespace control {
		namespace rosTools {
			static ros::Time convertToRosTime(uint64_t timestampNs) {
				ros::Time t;
				t.sec = static_cast<double>(timestampNs) / NS_PER_SEC;
				t.nsec = timestampNs % static_cast<uint64_t>(1e9);
				return t;
			}
		};
	};
};



#endif /* EEROS_ROS_TOOLS_HPP */