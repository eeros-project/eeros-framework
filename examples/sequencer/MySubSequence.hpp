#pragma once

#include <eeros/sequencer/Sequence.hpp>

/** Dies ist eine Beispielklasse für den Anwender. Damit wird gezeigt, wie dieser Sequnecer verwendet wird
  *
  */
class MySubSequence : public Sequence
{
public:
	MySubSequence(double period, string name);

	//Destruktor muss virtual sein, damit er automatisch aufgerufen wird.
	virtual ~MySubSequence(void);

	/**eigene Steps füllen, Init wird automatisch immer hinzugefügt von Sequence
	 * der nächste sollte Initialising sein, Rest spielt keine Rolle
	 */
	void fillSequencerSteps();
};

