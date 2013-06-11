#pragma once

#include <list>
using namespace std;

class SequencerStep;

class Transitions
{
private:
	list<string> allowedSteps;
	void deleteSequencerStepsName();

public:
	Transitions();
	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~Transitions(void);
	void addSequencerStepName(string name);
	bool isTransitionAllowed(string name);
};

