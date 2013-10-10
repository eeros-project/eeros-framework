#ifndef ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_
#define ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_

#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>

class SafetyContext;
class SafetySystem;

enum { kInvalidLevel = -1 };
enum EventType { kPrivateEvent, kPublicEvent };

class SafetyLevel {

	friend class SafetySystem;

public:
	SafetyLevel(int32_t id, std::string description);
	virtual ~SafetyLevel();
	
	int32_t getId();
	int32_t getLevelIdForEvent(uint32_t event, bool privateEventOk = false);
	
	void addEvent(uint32_t event, int32_t nextLevelId, EventType type = kPrivateEvent);
	
	void setInputAction(InputAction* action);
	void setInputActions(std::vector<InputAction*> actionList);
	void setOutputAction(OutputAction* action);
	void setOutputActions(std::vector<OutputAction*> actionList);
	
	void setLevelAction(std::function<void (SafetyContext* context)> action);
	
	SafetyLevel& operator<(const SafetyLevel&);
	
private:
	std::function<void (SafetyContext*)> action;
	
	int32_t id;
	std::string description;
	std::map<uint32_t, std::pair<int32_t, EventType>> transitions;
	
	std::vector<InputAction*> inputAction;
	std::vector<OutputAction*> outputAction;
};

#endif // ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_
