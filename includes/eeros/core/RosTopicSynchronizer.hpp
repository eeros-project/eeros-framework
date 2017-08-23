#ifndef ORG_EEROS_CORE_ROS_TOPIC_SYNCHRONIZER_HPP_
#define ORG_EEROS_CORE_ROS_TOPIC_SYNCHRONIZER_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <list>

namespace eeros {

	class RosTopicSynchronizer {
	public:
		RosTopicSynchronizer();
		
		template <class rosTopicType>
		void addTopic(std::string rosTopic);
		
		template <class rosTopicType2>
		void callback(const rosTopicType2 msg);
	
	
	private:
		ros::NodeHandle syncNodeHandler;
		ros::CallbackQueue syncCallbackQueue;
// 		std::list
	
	};
}

#endif // ORG_EEROS_CORE_ROS_TOPIC_SYNCHRONIZER_HPP_