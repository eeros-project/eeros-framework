
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Init.hpp>


Sequence::Sequence(double period, string name) :
	Executor(period),
	sequenceName(name)
{
	Transitions* trans = new Transitions();
	trans->addAllowedTransitionName("Initialising");
	Init* init = new Init(trans, "Init", this);
	currentStep = init;
}

Sequence::~Sequence(void)
{
	//deleteBlockList();
	deleteSequencerStepList();
	deleteSubSequences();
}


/*void Sequence::deleteBlockList(){
	list<Block*>::iterator iter = blockes.begin();
	while(iter != blockes.end()){
		delete (*iter);
		iter++;
	}
	blockes.clear();
}
*/

void Sequence::deleteSequencerStepList(){
	list<SequencerStep*>::iterator iter = sequencerSteps.begin();
	SequencerStep* step = 0;
	while(iter != sequencerSteps.end()){
		step = *iter;
		delete step;
		iter++;
	}
	sequencerSteps.clear();
}

/*list<Block*>& Sequence::getBlockList(){
	return blockes;
}
*/

void Sequence::safeTransition(string nameOfDesiredSequenceStep){
	if(nameOfDesiredSequenceStep.compare("") == 0){
		//wird von Stopping aufgerufen
		//Achtung: currentStep muss erhalten bleiben, bis der Thread wirklich beendet wurde.
		//currentStep = 0;
		return;
	}
	if( currentStep->getTransitons()->isTransitionAllowed(nameOfDesiredSequenceStep) ){
		list<SequencerStep*>::iterator iter = sequencerSteps.begin();
		while(iter != sequencerSteps.end()){
			if((*iter)->getName().compare(nameOfDesiredSequenceStep) == 0){
				currentStep = *iter;
				return;
			}
			iter++;
		}
	}
}

/** run wird in einer while Schleife vom ExecutorService aufgerufen
 *  private Methoden überschreiben geht in C++
 */
void Sequence::run(){
	if(currentStep){
		currentStep->run();
	}
}

void Sequence::deleteSubSequences(){
	list<Sequence*>::iterator iter = subSequences.begin();
	Sequence* sequence = 0;
	while(iter != subSequences.end()){
		sequence = *iter;
		delete sequence;
		iter++;
	}
	subSequences.clear();
}

void Sequence::addSubSequence(Sequence* subSequence){
	subSequences.push_back(subSequence);
}

Sequence* Sequence::findSequence(string name){
	list<Sequence*>::iterator iter = subSequences.begin();
		while(iter != subSequences.end()){
			if((*iter)->getName().compare(name) == 0){
				return *iter;
			}
			iter++;
		}
		return 0;
}

void Sequence::deleteSequence(string name){
	list<Sequence*>::iterator iter = subSequences.begin();
		while(iter != subSequences.end()){
			if((*iter)->getName().compare(name) == 0){
				subSequences.erase(iter);
				return;
			}
			iter++;
		}
}

string Sequence::getName(){
	return sequenceName;
}

void Sequence::addSequencerStep(SequencerStep* sequencerStep){
	sequencerSteps.push_back(sequencerStep);
}