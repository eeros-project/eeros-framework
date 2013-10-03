#ifndef ORG_EEROS_SAFETY_SAFETYSTATE_HPP_
#define ORG_EEROS_SAFETY_SAFETYSTATE_HPP_

#include <vector>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/hal/HAL.hpp>

class SafetyState {
public:
	SafetyState();
	SafetyLevel& getLevel(uint32_t levelId);
	
	std::vector<SafetyLevel> levels;
	SafetyLevel* currentLevel;
	
	std::vector<SystemOutputInterface*> criticalOutputs;
	std::vector<SystemInputInterface*> criticalInputs;
};

#endif // ORG_EEROS_SAFETY_SAFETYSTATE_HPP_