#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_

#include <eeros/core/System.hpp>
#include <eeros/control/ROS/RosPublisher.hpp>
#include <sensor_msgs/FluidPressure.h>

namespace eeros {
	namespace control {

		class RosPublisherDouble : public RosPublisher<sensor_msgs::FluidPressure::Type, double> {
			typedef sensor_msgs::FluidPressure::Type TRosMsg;
		public:
			RosPublisherDouble (ros::NodeHandle& rosNodeHandler, const std::string& topic, const uint32_t queueSize=1000) :
				RosPublisher<TRosMsg, double>(rosNodeHandler, topic, queueSize) { }
			
			void setRosMsg(TRosMsg& msg) {
				msg.fluid_pressure = in.getSignal().getValue();
				msg.header.stamp.sec = eeros::System::getTime();
				msg.header.stamp.nsec = eeros::System::getTimeNs() % static_cast<uint64_t>(1e9);
			}
		};
	};
};

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_ */
