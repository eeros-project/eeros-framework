#ifndef ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_DOUBLE_HPP_
#define ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_DOUBLE_HPP_

#include <eeros/core/System.hpp>
#include <eeros/control/ROS/RosBlockPublisher.hpp>
#include <sensor_msgs/FluidPressure.h>

using namespace eeros::control;


class RosBlockPublisherDouble : public RosBlockPublisher<sensor_msgs::FluidPressure::Type> {
	typedef sensor_msgs::FluidPressure::Type		TRosMsg;

public:
	RosBlockPublisherDouble (	ros::NodeHandle& rosNodeHandler,
								const std::string& topic,
								const uint32_t queueSize=1000) :
		RosBlockPublisher<TRosMsg>( rosNodeHandler, topic, queueSize )
		{}
	
	void setRosMsg(TRosMsg& msg) {
		msg.fluid_pressure = in.getSignal().getValue();
		msg.header.stamp.sec = eeros::System::getTime();
		msg.header.stamp.nsec = eeros::System::getTimeNs() % static_cast<uint64_t>(1e9);
	}
	
	Input<double>& getIn()	{return in;  };
	
	
protected:
	Input<double> in;

};


#endif /* ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_DOUBLE_HPP_ */
