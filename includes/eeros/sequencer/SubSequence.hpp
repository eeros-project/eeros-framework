#pragma once

/** SubSequence ist eine Untersequenz, welche anstatt eines Steps aufgerufen werden kann.
*/

class SubSequence : public Sequence
{
public:
	SubSequence(void);
	~SubSequence(void);
};

