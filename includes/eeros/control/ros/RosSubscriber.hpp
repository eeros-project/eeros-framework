#ifndef ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_
#define ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eeros/control/Block1o.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
	namespace control {
		template < typename TRosMsg, typename SigOutType >
		class RosSubscriber : public Block1o<SigOutType> {

		public:
			RosSubscriber(const std::string& topic, const uint32_t queueSize=1000, const bool callNewest=false) :
				topic (topic),
				callNewest(callNewest)
			{
				ros::NodeHandle handle;
				subscriber = handle.subscribe(topic, queueSize, &RosSubscriber::rosCallbackFct, this);
				ROS_DEBUG_STREAM("RosBlockSubscriber, reading from topic: '" << topic << "' created.");
			}

			virtual void rosCallbackFct(const TRosMsg& msg) = 0;

// 			void rosCallbackFct(const TRosMsg& msg) {
// 
// 				// USER DEFINED: 1.) Set timestamp for all outputs
// 				//               2.) Get the data from the message
// 				//               3.) Cast the data if necessary
// 				//               4.) Insert the data into output
// 
// 				auto time = eeros::System::getTimeNs();
// 				this->out.getSignal().setTimestamp( time );
// 
// 				this->out.getSignal().setValue(static_cast< TOutput >( msg.data) );
// 			}

			virtual void run() {
				if (callNewest)
					ros::getGlobalCallbackQueue()->callAvailable();		// calls callback fct. for all available messages.
				else													//  Only newest message is processed. Older ones are discarded.
					ros::getGlobalCallbackQueue()->callOne();			// calls callback fct. only for the oldest message
			}


		protected:
			ros::Subscriber subscriber;
			const std::string& topic;
			bool callNewest;

		private:
		};
	}
}

#endif /* ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_ */
