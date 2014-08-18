#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/core/EEROSException.hpp>

#include <sstream>

namespace eeros {
	namespace safety {

		SafetyProperties::SafetyProperties() : entryLevel(-1) {
			// nothing to do
		}
		
		SafetyProperties::~SafetyProperties() {
			// nothing to do
		}

		SafetyLevel& SafetyProperties::level(uint32_t levelId) {
			for(auto& l : levels) {
				if(l.getId() == levelId) return l;
			}
			std::stringstream msg;
			msg << "safety level with id '" << levelId << "' not defined!";
			throw EEROSException(msg.str());
		}
		
		SafetyLevel* SafetyProperties::entryLevelPtr() {
			return &level(entryLevel);
		}
		
		bool SafetyProperties::verify() {
			bool check = true;
			
			// Check in every level
			for(auto& l : levels) {
				// if the output action of every output is defined
				// TODO
				
				// if the input action for every critical input is defined
				// TODO
			}
			
			// Check entry level
			check = entryLevelPtr() != nullptr;
			
			return check;
		}
		
		void SafetyProperties::addEventToLevel(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type) {
			level(levelId).addEvent(event, nextLevelId, type);
		}

		void SafetyProperties::addEventToLevelAndAbove(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type) {
			for(auto& level : levels) {
				if(level.getId() >= levelId) level.addEvent(event, nextLevelId, type);
			}
		}

		void SafetyProperties::addEventToLevelAndBelow(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type) {
			for(auto& level : levels) {
				if(level.getId() <= levelId) level.addEvent(event, nextLevelId, type);
			}
		}

		void SafetyProperties::addEventToAllLevelsBetween(int32_t lowerLevelId, int32_t upperLevelId, uint32_t event, int32_t nextLevelId, EventType type) {
			for(auto& level : levels) {
				if(level.getId() >= lowerLevelId && level.getId() <= upperLevelId) level.addEvent(event, nextLevelId, type);
			}
		}
		
	};
};
