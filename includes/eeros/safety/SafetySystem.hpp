#ifndef ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
#define ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
#include <string>
#include <list>
#include <vector>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/core/Runnable.hpp>

class Event;
class SystemInput;
class SystemOutput;

class SafetySystem : public Runnable {

public:
	SafetyLevel& operator[](unsigned levelId);
	
	void setSafetyLevels(std::vector<SafetyLevel>& levels);
	SafetyLevel& level(uint32_t levelId);
	void addEventToAllLevelsAbove(uint32_t level, uint32_t event, uint32_t nextLevel);
//	void addCriticalOutput(std::string output);
//	void addCriticalInput(std::string output, Event* event);
	
	void run();
	
	static SafetySystem& instance();
	
private:
	SafetySystem();
	SafetySystem(const SafetySystem&);
	SafetySystem& operator=(const SafetySystem&);
	
	std::vector<SafetyLevel> levels;
	std::list<std::string> criticalOutputs;
	std::list<std::string> criticalInputs;
	
	static SafetySystem safetySystem;
};

#endif // ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
