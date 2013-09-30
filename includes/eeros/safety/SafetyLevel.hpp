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

//enum Action { notDefined, checkTrue, checkFalse, ignore, checkRange, setTrue, setFalse, leave, setToValue };

// class InputAction {
// public:
// 	InputAction(Action action);
// 	InputAction(Action action, double lowerLimit, double upperLimit);
// };
// 
// class OutputAction {
// public:
// 	OutputAction(Action action);
// 	OutputAction(Action action, double value);
// 	OutputAction(Action action, double lowerLimit, double upperLimit);
// };

class SafetyLevel {

	friend class SafetySystem;

public:
	SafetyLevel(uint32_t id, std::string description);
	
// 	SafetyLevel(uint32_t id, std::string description,
// 				std::map<std::string, InputAction> inputStates,
// 				std::map<std::string, OutputAction> outputStates);
	
	virtual ~SafetyLevel();
	
	uint32_t getId();
	uint32_t getLevelIdForEvent(uint32_t event, bool privateEventOk = false);
	
	void addEvent(uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	
	void setInputAction(InputAction action);
	void setInputActions(std::vector<InputAction> actionList);
	void setOutputAction(OutputAction action);
	void setOutputActions(std::vector<OutputAction> actionList);
	
	void setLevelAction(std::function<void (SafetyContext* context)> action);
	
	SafetyLevel& operator<(const SafetyLevel&);
	
private:
	std::function<void (SafetyContext*)> action;
	
	uint32_t id;
	std::string description;
	std::map<uint32_t, uint32_t> transitions;
	
	std::vector<InputAction> inputAction;
	std::vector<OutputAction> outputAction;
};

#endif // ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_
