#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/core/EEROSException.hpp>

#include <sstream>

namespace eeros {
	namespace safety {

		SafetyProperties::SafetyProperties() : entryLevel(nullptr) {
			// nothing to do
		}
		
		SafetyProperties::~SafetyProperties() {
			// nothing to do
		}
		
		SafetyLevel* SafetyProperties::getEntryLevel() {
			return entryLevel;
		}
		
		void SafetyProperties::setEntryLevel(SafetyLevel& entryLevel) {
			this->entryLevel = &entryLevel;
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
			check = getEntryLevel() != nullptr;
			
			return check;
		}
		
		void SafetyProperties::addEventToLevel(SafetyLevel& level, SafetyEvent event, SafetyLevel& nextLevel, EventType type) {
			level.addEvent(event, nextLevel, type);
		}

		void SafetyProperties::addEventToLevelAndAbove(SafetyLevel& level, SafetyEvent event, SafetyLevel& nextLevel, EventType type) {
			for(auto& lev : levels) {
				if(lev->id >= level.id) lev->addEvent(event, nextLevel, type);
			}
		}

		void SafetyProperties::addEventToLevelAndBelow(SafetyLevel& level, SafetyEvent event, SafetyLevel& nextLevel, EventType type) {
			for(auto& lev : levels) {
				if(lev->id <= level.id) lev->addEvent(event, nextLevel, type);
			}
		}

		void SafetyProperties::addEventToAllLevelsBetween(SafetyLevel& lowerLevel, SafetyLevel& upperLevel, SafetyEvent event, SafetyLevel& nextLevel, EventType type) {
			for(auto& lev : levels) {
				if(lev->id >= lowerLevel.id && lev->id <= upperLevel.id) lev->addEvent(event, nextLevel, type);
			}
		}
		
	};
};
