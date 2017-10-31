#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLEARRAY_HPP
#define ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLEARRAY_HPP

#include <eeros/control/ROS/RosBlockPublisher.hpp>
#include <eeros/math/Matrix.hpp>
#include <std_msgs/Float64MultiArray.h>

namespace eeros {
	namespace control {

		template < typename SigInType >
		class RosPublisherDoubleArray : public RosBlockPublisher<std_msgs::Float64MultiArray::Type> {
			typedef std_msgs::Float64MultiArray::Type TRosMsg;
		public:
			RosPublisherDoubleArray(ros::NodeHandle& rosNodeHandler, const std::string& topic, const uint32_t queueSize=1000) :
				RosBlockPublisher<TRosMsg>(rosNodeHandler, topic, queueSize) { }
				
			void setRosMsg(TRosMsg& msg) {
				if (this->in.isConnected()) {
					auto val = this->in.getSignal().getValue();
					auto valTmpDouble = val.getColVector(0);
					msg.data = valTmpDouble;
				}
			}
			
			Input<SigInType>& getIn() {return in;}
				
		protected:
			Input<SigInType> in;
		};

	};
};

#endif // ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLEARRAY_HPP