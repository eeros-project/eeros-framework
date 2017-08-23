#include <eeros/core/RosTopicSynchronizer.hpp>


using namespace eeros;

// namespace {
	
RosTopicSynchronizer::RosTopicSynchronizer() {
		syncNodeHandler.setCallbackQueue(&syncCallbackQueue);
}

// 	template <class rosTopicType>
// 		void addTopic(std::string rosTopic);

void RosTopicSynchronizer::addTopic(std::__cxx11::string rosTopic) {
	syncNodeHandler.subscribe(rosTopic, 1, &callback<rosTopicType>());
}
// 	rosTopicType a;
// }

void RosTopicSynchronizer::callback(const rosTopicType2) {
	return;
}






// }