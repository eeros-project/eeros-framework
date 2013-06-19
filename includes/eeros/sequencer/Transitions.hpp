#pragma once

#include <string>
#include <list>

class SequencerStep;

class Transitions
{
	friend class SequencerStep;
private:
	std::list<std::string> allowedSteps;
	void deleteSequencerStepsName();

public:
	Transitions();
	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~Transitions(void);
	void addAllowedTransitionName(std::string name);
	bool isTransitionAllowed(std::string name);
};

