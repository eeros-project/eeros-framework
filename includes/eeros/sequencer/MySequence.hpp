#pragma once

#include "Sequence.hpp"

/** Dies ist eine Beispielklasse für den Anwender. Damit wird gezeigt, wie dieser Sequnecer verwendet wird
  *
  */
class MySequence : public Sequence
{
public:
	MySequence(double period, string name);

	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~MySequence(void);

	/**eigene Steps füllen, Init wird automatisch immer hinzugefügt von Sequence
	 * der nächste sollte Initialising sein, Rest spielt keine Rolle
	 */
	void fillSequencerSteps();
};

