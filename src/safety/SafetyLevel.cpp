#include <eeros/safety/SafetyLevel.hpp>

namespace eeros {
	namespace safety {

		SafetyEvent::SafetyEvent(std::string description) : description(description) {
			static int count = 0;
			id = count++;
		}

		SafetyEvent::~SafetyEvent() {
		}

		std::string SafetyEvent::getDescription() {
			return description;
		}

		SafetyLevel::SafetyLevel(std::string description) : description(description) {
			// number the levels when adding them to the safety system
		}

		SafetyLevel::~SafetyLevel() {
			// nothing to do...
		}

		bool SafetyLevel::operator<(const SafetyLevel& level) {
			return this->id < level.id;
		}

		bool SafetyLevel::operator<=(const SafetyLevel& level) {
			return this->id <= level.id;
		}

		bool SafetyLevel::operator>(const SafetyLevel& level) {
			return this->id > level.id;
		}

		bool SafetyLevel::operator>=(const SafetyLevel& level) {
			return this->id >= level.id;
		}

		bool SafetyLevel::operator==(const SafetyLevel& level) {
			return this->id == level.id;
		}

		bool SafetyLevel::operator!=(const SafetyLevel& level) {
			return this->id != level.id;
		}

		std::string SafetyLevel::getDescription() {
			return description;
		}

		uint32_t SafetyLevel::getNofActivations() {
			return nofActivations;
		}

		SafetyLevel* SafetyLevel::getDestLevelForEvent(SafetyEvent event, bool privateEventOk) {
			auto it = transitions.find(event.id);
			if(it != transitions.end()) {
				if((it->second.second != kPrivateEvent) || privateEventOk) return it->second.first;
			}
			return nullptr;
		}

		void SafetyLevel::addEvent(SafetyEvent event, SafetyLevel& nextLevel, EventType type) {
			transitions.insert(std::make_pair(event.id, std::make_pair(&nextLevel, type)));
		}

		void SafetyLevel::setLevelAction(std::function<void (SafetyContext*)> action) {
			this->action = action;
		}


		void SafetyLevel::setInputAction(InputAction* action) {
			inputAction.push_back(action);
		}

		void SafetyLevel::setInputActions(std::vector<InputAction*> actionList) {
			inputAction = actionList;
		}

		void SafetyLevel::setOutputAction(OutputAction* action) {
			outputAction.push_back(action);
		}

		void SafetyLevel::setOutputActions(std::vector<OutputAction*> actionList) {
			outputAction = actionList;
		}

		/********** Print functions **********/
		std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyEvent& event) {
			os << event.getDescription();
			return os;
		}

		std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyEvent* event) {
			os << event->getDescription();
			return os;
		}

		std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyLevel& level) {
			os << level.getDescription();
			return os;
		}

		std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyLevel* level) {
			os << level->getDescription();
			return os;
		}

	};
};
