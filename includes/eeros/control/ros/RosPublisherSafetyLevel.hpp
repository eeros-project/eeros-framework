#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_

#include <eeros/control/ros/RosPublisher.hpp>
#include <std_msgs/UInt32.h>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/Block.hpp>

using namespace eeros::safety;

namespace eeros {
	namespace control {

		class RosPublisherSafetyLevel : public Block {
			typedef std_msgs::UInt32::Type TRosMsg;
		public:
			RosPublisherSafetyLevel(const std::string& topic, const uint32_t queueSize=1000) :
				topic (topic)
			{
				ros::NodeHandle handle;
				publisher = handle.advertise<TRosMsg>(topic, queueSize);
				ROS_DEBUG_STREAM("RosPublisherSafetyLevel, reading from topic: '" << topic << "' created.");
			}
			
			void setSafetySystem(SafetySystem& ss) {
				safetySystem = &ss;
			}
			
			void run() {
				if (safetySystem != nullptr) {
					SafetyLevel sl = safetySystem->getCurrentLevel();
					msg.data = sl.getLevelId();
					publisher.publish(msg);
				}
			}
			
		protected:
			ros::Publisher publisher;
			const std::string& topic;
			TRosMsg msg;
			SafetySystem* safetySystem;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_ */
