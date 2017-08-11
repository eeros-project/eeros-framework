#ifndef ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_DOUBLE_HPP_
#define ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_DOUBLE_HPP_

#include <eeros/control/RosBlockPublisher.hpp>
#include <std_msgs/Float64.h>

using namespace eeros::control;


class RosBlockPublisherDouble : public RosBlockPublisher<std_msgs::Float64::Type> {
	typedef std_msgs::Float64::Type			TRosMsg;

public:
	RosBlockPublisherDouble (	ros::NodeHandle& rosNodeHandler,
								const std::string& topic,
								const uint32_t queueSize=1000) :
		RosBlockPublisher<TRosMsg>( rosNodeHandler, topic, queueSize )
		{}
	
	void setRosMsg(TRosMsg& msg) {
		msg.data = in.getSignal().getValue();
	}
	
	Input<double>& getIn()	{return in;  };
	
	
protected:
	Input<double> in;

};


#endif /* ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_DOUBLE_HPP_ */