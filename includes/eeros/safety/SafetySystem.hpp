#ifndef ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
#define ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_

#include <string>
#include <vector>
#include <eeros/core/Runnable.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetyContext.hpp>
#include <eeros/safety/SafetyState.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/hal/HAL.hpp>

namespace eeros {
	namespace safety {

		class SafetySystem : public Runnable {
		public:
			SafetyLevel& getCurrentLevel(void);
			SafetyLevel& getLevel(int32_t levelId);
			SafetyLevel& operator[](unsigned levelId);
			
			void defineSafetyLevels(std::vector<SafetyLevel> levels);
			void setEntryLevel(int32_t levelId);

			void defineCriticalInputs(std::vector<eeros::hal::SystemInputInterface*> inputs);
			void defineCriticalOutputs(std::vector<eeros::hal::SystemOutputInterface*> outputs);
			void addCriticalInput(eeros::hal::SystemInputInterface& input);
			void addCriticalOutput(eeros::hal::SystemOutputInterface& output);
			
			void addEventToLevel(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type = kPrivateEvent);
			void addEventToLevelAndAbove(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type = kPrivateEvent);
			void addEventToLevelAndBelow(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type = kPrivateEvent);
			void addEventToAllLevelsBetween(int32_t lowerLevelId, int32_t upperLevelId, uint32_t event, int32_t nextLevelId, EventType type = kPrivateEvent);
			
			void triggerEvent(uint32_t event);
			
			void run();
			
			static SafetySystem& instance();
			
		private:
			SafetySystem();
			SafetySystem(const SafetySystem&);
			SafetySystem& operator=(const SafetySystem&);
			
		//	std::vector<SafetyLevel> levels;
		//	SafetyLevel* currentLevel;
			
		// 	std::vector<SystemOutputInterface*> criticalOutputs;
		// 	std::vector<SystemInputInterface*> criticalInputs;
			
			SafetyContext privateContext;
			SafetyState state;
		};

	};
};

#endif // ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
