#pragma once

/** MainSequence ist die Hauptsequenz
*/

using namespace std;

class MainSequence : public Sequence
{
private:
	/** Liste aller SubSequence, welche in einen Step aufgerufen wurden
	  *
	  */
	list<Sequence*> subSequences;
	void deleteSubSequences();

public:
	MainSequence(void);
	~MainSequence(void);
	void addSubSequence(SubSequence* subSequence);
};

