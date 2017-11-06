#ifndef ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLEARRAY_HPP
#define ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLEARRAY_HPP

#include <eeros/control/ros/RosSubscriber.hpp>
#include <eeros/core/System.hpp>
#include <std_msgs/Float64MultiArray.h>

namespace eeros {
	namespace control {

		template < typename SigOutType >
		class RosSubscriberDoubleArray : public RosSubscriber<std_msgs::Float64MultiArray::Type, SigOutType> {
			typedef std_msgs::Float64MultiArray::Type TRosMsg;
		public:
			RosSubscriberDoubleArray(const std::string& topic, const uint32_t queueSize=1000, const bool callNewest=false ) :
				RosSubscriber<TRosMsg, SigOutType>(topic, queueSize, callNewest){ }
				
			void rosCallbackFct(const TRosMsg& msg) {
				auto time = eeros::System::getTimeNs();	// use system time for timestamp
				this->out.getSignal().setTimestamp( time );
				std::vector<double> valTmp(msg.data.begin(), msg.data.end() );
				val.setCol(0, valTmp);
				this->out.getSignal().setValue(val);
			}
		protected:
			SigOutType val;
		};
	};
};

#endif // ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLEARRAY_HPP