#ifndef ORG_EEROS_CORE_ROS_TOPIC_SYNCHRONIZER_HPP_
#define ORG_EEROS_CORE_ROS_TOPIC_SYNCHRONIZER_HPP_

#include <rclcpp/rclcpp.hpp>
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
		rclcpp::Node::SharedPtr syncNodeHandler;
		rclcpp::CallbackGroup::SharedPtr syncCallbackQueue;
	};
}

#endif // ORG_EEROS_CORE_ROS_TOPIC_SYNCHRONIZER_HPP_
