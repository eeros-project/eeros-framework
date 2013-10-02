#ifndef ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
#define ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_

#include <string>
#include <vector>
#include <eeros/core/Runnable.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetyContext.hpp>
#include <eeros/safety/SafetyState.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/hal/HAL.hpp>

class SafetySystem : public Runnable {
public:
	SafetyLevel& getLevel(uint32_t levelId);
	SafetyLevel& operator[](unsigned levelId);
	
	void defineSafetyLevels(std::vector<SafetyLevel> levels);
	void setEntryLevel(uint32_t levelId);

	void defineCriticalInputs(std::vector<SystemInputInterface*> inputs);
	void defineCriticalOutputs(std::vector<SystemOutputInterface*> outputs);
	
	void addEventToLevel(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	void addEventToLevelAndAbove(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	void addEventToLevelAndBelow(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	void addEventToAllLevelsBetween(uint32_t lowerLevelId, uint32_t upperLevelId, uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	
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

#endif // ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
