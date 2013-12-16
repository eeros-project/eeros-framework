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
			
			void addEventToLevel(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type);
			void addEventToLevelAndAbove(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type);
			void addEventToLevelAndBelow(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type);
			void addEventToAllLevelsBetween(int32_t lowerLevelId, int32_t upperLevelId, uint32_t event, int32_t nextLevelId, EventType type);
			SafetyLevel* entryLevelPtr();
			bool verify();
			
		protected:
			virtual SafetyLevel& level(uint32_t levelId);
			
			std::vector<SafetyLevel> levels;
			std::vector<eeros::hal::SystemOutputInterface*> criticalOutputs;
			std::vector<eeros::hal::SystemInputInterface*> criticalInputs;
			uint32_t entryLevel;
		};
		
	};
};

#endif // ORG_EEROS_SAFETY_SAFETYPROPORTIES_HPP_
