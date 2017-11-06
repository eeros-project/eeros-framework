#ifndef ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP
#define ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP

#include <eeros/control/ros/RosSubscriber.hpp>
#include <eeros/core/System.hpp>
#include <std_msgs/Float64.h>

namespace eeros {
	namespace control {

		class RosSubscriberDouble : public RosSubscriber<std_msgs::Float64::Type, double> {
			typedef std_msgs::Float64::Type TRosMsg;
		public:
			RosSubscriberDouble(const std::string& topic, const uint32_t queueSize=1000, const bool callNewest=false ) :
				RosSubscriber<TRosMsg, double>(topic, queueSize, callNewest){ }
				
			void rosCallbackFct(const TRosMsg& msg) {
				auto time = eeros::System::getTimeNs();	// use system time for timestamp
				this->out.getSignal().setTimestamp( time );
				this->out.getSignal().setValue(static_cast<double>(msg.data) );
			}
		};
	};
};

#endif // ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP