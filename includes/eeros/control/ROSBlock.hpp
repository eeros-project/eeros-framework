#ifndef ORG_EEROS_CONTROL_ROSBLOCK_HPP_
#define ORG_EEROS_CONTROL_ROSBLOCK_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eeros/control/Block1o.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
	namespace control {
		template < typename TMsg >
		class ROSBlock : public Block {

		public:
			ROSBlock(ros::NodeHandle& rosNodeHandler, const std::string& topic, uint32_t queueSize=1000) :
				rosNodeHandler(rosNodeHandler),
				topic (topic)
//				rosCallbackFctFinished( false )
			{
				subscriber = rosNodeHandler.subscribe(topic, queueSize, &ROSBlock::rosCallbackFct, this);
				ROS_DEBUG_STREAM("ROSBlock, reading from topic: '" << topic << "' created.");
			}


			void rosCallbackFctBase(const TMsg& msg) {
				rosCallbackFct(msg);
			}

			virtual void rosCallbackFct(const TMsg& msg) = 0;

			virtual void run() {
	//			ros::getGlobalCallbackQueue()->callAvailable();		// calls callback fct. for all available messages
				ros::getGlobalCallbackQueue()->callOne();			// calls callback fct. only for the oldest message
			}


		protected:
			//ROS variables
			ros::NodeHandle& rosNodeHandler;
			ros::Subscriber subscriber;
			const std::string& topic;

		private:
		};
	}
}

#endif /* ORG_EEROS_CONTROL_ROSBLOCK_HPP_ */
