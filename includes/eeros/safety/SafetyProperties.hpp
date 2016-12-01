#ifndef ORG_EEROS_SAFETY_SAFETYPROPORTIES_HPP_
#define ORG_EEROS_SAFETY_SAFETYPROPORTIES_HPP_

#include <stdint.h>
#include <vector>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/hal/HAL.hpp>

namespace eeros {
	namespace safety {
		
		// Forward declarations
		class SafetySystem;
		
		class SafetyProperties {
			
			friend class SafetySystem;
			
		public:
			SafetyProperties();
			virtual ~SafetyProperties();
			
			void addEventToLevel(SafetyLevel& level, SafetyEvent event, SafetyLevel& nextLevel, EventType type);
			void addEventToLevelAndAbove(SafetyLevel& level, SafetyEvent event, SafetyLevel& nextLevel, EventType type);
			void addEventToLevelAndBelow(SafetyLevel& level, SafetyEvent event, SafetyLevel& nextLevel, EventType type);
			void addEventToAllLevelsBetween(SafetyLevel& lowerLevel, SafetyLevel& upperLevel, SafetyEvent event, SafetyLevel& nextLevel, EventType type);
			SafetyLevel* getEntryLevel();
			bool verify();
			void addLevel(SafetyLevel& level) {levels.push_back(&level);}
		protected:
			void setEntryLevel(SafetyLevel& entryLevel);
			
			std::function<void (SafetyContext*)> exitFunction;			
			std::vector<SafetyLevel*> levels;
			std::vector<eeros::hal::OutputInterface*> criticalOutputs;
			std::vector<eeros::hal::InputInterface*> criticalInputs;
		private:
			SafetyLevel* entryLevel;
		};
		
	};
};

#endif // ORG_EEROS_SAFETY_SAFETYPROPORTIES_HPP_
