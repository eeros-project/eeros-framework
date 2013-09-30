#ifndef ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
#define ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_

#include <string>
#include <vector>
#include <eeros/core/Runnable.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/hal/HAL.hpp>

template <typename T>
SetOutputAction<T> set(SystemOutput<T>& output, T value) {
	return SetOutputAction<T>(output, value);
}

template <typename T>
LeaveOutputAction<T> leave(SystemOutput<T>& output) {
	return LeaveOutputAction<T>(output);
}

template <typename T>
IgnoreInputAction<T> ignore(SystemInput<T>& input) {
	return IgnoreInputAction<T>(input);
}

template <typename T>
CheckInputAction<T> check(SystemInput<T>& input, T value, uint32_t event) {
	return CheckInputAction<T>(input, value, event);
}

template <typename T>
CheckRangeInputAction<T> range(SystemInput<T>& input, T min, T max, uint32_t event) {
	return CheckRangeInputAction<T>(input, min, max, event);
}

class SafetyContext {
	friend class SafetySystem;
private:
	SafetyContext() {}
};

class SafetySystem : public Runnable {
public:
	SafetyLevel& getLevel(uint32_t levelId);
	SafetyLevel& operator[](unsigned levelId);
	
	void defineSafetyLevels(std::vector<SafetyLevel> levels);
	void setEntryLevel(uint32_t levelId);

//	void defineCriticalInputs(std::vector<SystemInput> inputs);
//	void defineCriticalOutputs(std::vector<SystemOutput> outputs);
	
	void addEventToLevel(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	void addEventToLevelAndAbove(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	void addEventToLevelAndBelow(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	void addEventToAllLevelsBetween(uint32_t lowerLevelId, uint32_t upperLevelId, uint32_t event, uint32_t nextLevelId, EventType type = kPrivateEvent);
	
	void triggerEvent(uint32_t event, SafetyContext* context = 0);
	
	void run();
	
	static SafetySystem& instance();
	
private:
	SafetySystem();
	SafetySystem(const SafetySystem&);
	SafetySystem& operator=(const SafetySystem&);
	
	std::vector<SafetyLevel> levels;
	SafetyLevel* currentLevel;
	
	std::vector<std::string> criticalOutputs;
	std::vector<std::string> criticalInputs;
	
	SafetyContext* privateContext;
};

#endif // ORG_EEROS_SAFETY_SAFETYSYSTEM_HPP_
