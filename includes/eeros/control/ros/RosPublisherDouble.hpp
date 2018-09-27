#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_

#include <eeros/core/System.hpp>
#include <eeros/control/ros/RosPublisher.hpp>
#include <std_msgs/Float64.h>

namespace eeros {
	namespace control {

		class RosPublisherDouble : public RosPublisher<std_msgs::Float64::Type, double> {
			typedef std_msgs::Float64::Type TRosMsg;
		public:
			RosPublisherDouble (const std::string& topic, const uint32_t queueSize=1000) :
				RosPublisher<TRosMsg, double>(topic, queueSize) { }
			
			void setRosMsg(TRosMsg& msg) {
				msg.data = in.getSignal().getValue();
// 				msg.header.stamp.sec = eeros::System::getTime();
// 				msg.header.stamp.nsec = eeros::System::getTimeNs() % static_cast<uint64_t>(1e9);
			}
		};
	};
};

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_ */
