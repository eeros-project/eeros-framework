#ifndef ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_
#define ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_

#include <stdint.h>
#include <string>
#include <map>
#include <functional>

class Event;

class SafetyLevel {

public:
	SafetyLevel(uint32_t id, std::string description, std::map<std::string, bool> outputStates = std::map<std::string, bool>());
	virtual ~SafetyLevel();

	void addEvent(uint32_t event, uint32_t nextLevel);
	void setExitEvent(uint32_t event);
	void setOutputState(std::string output, double value);
	void setOutputState(std::string output, bool value);
	void setLevelAction(std::function<void (void)> action);
	
	uint32_t getId();
	
	SafetyLevel& operator<(const SafetyLevel&);
	
private:
	std::function<void (void)> action;
	
	uint32_t id;
	std::string description;
	std::map<uint32_t, uint32_t> transitions;
	Event* exitEvent;
	
	std::map<std::string, bool> outputs;
};

#endif // ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_
