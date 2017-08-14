#ifndef ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_HPP_
#define ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>

namespace eeros {
	namespace control {
		template < typename TRosMsg >
		class RosBlockPublisher : public Block {

		public:
			RosBlockPublisher(ros::NodeHandle& rosNodeHandler, const std::string& topic, uint32_t queueSize=1000, bool callNewest=false) :
				rosNodeHandler(rosNodeHandler),
				topic (topic)
// 				callNewest(callNewest)
			{
				publisher = rosNodeHandler.advertise<TRosMsg>(topic, queueSize);
				ROS_DEBUG_STREAM("RosBlockPublisher, reading from topic: '" << topic << "' created.");
			}

			virtual void setRosMsg( TRosMsg& msg ) = 0;
			
			void run() {
				setRosMsg(msg);
				publisher.publish(msg);
			}


		protected:
			//ROS variables
			ros::NodeHandle& rosNodeHandler;
			ros::Publisher publisher;
			const std::string& topic;
// 			bool callNewest;
			TRosMsg msg;

		private:
		};
	}
}

#endif /* ORG_EEROS_CONTROL_ROSBLOCK_PUBLISHER_HPP_ */
