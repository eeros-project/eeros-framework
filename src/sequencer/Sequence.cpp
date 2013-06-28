#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceException.hpp>

std::list<eeros::sequencer::Sequence*> eeros::sequencer::Sequence::allSequences;

eeros::sequencer::Sequence::Sequence(std::string name, Sequencer& caller)
	: sequenceName(name),
      callerThread(caller){
		if(&callerThread != eeros::sequencer::Sequencer::getMainSequencer()){
			eeros::sequencer::Sequencer::getMainSequencer()->addSubSequencer(&caller);
		}
		eeros::sequencer::Sequence::allSequences.push_back(this);
}

std::string eeros::sequencer::Sequence::getName(){
	return sequenceName;
}

void eeros::sequencer::Sequence::addCallBack(eeros::sequencer::Sequence::method callback){
	callBacks.push_back(callback);
}

void eeros::sequencer::Sequence::run(){
	if(callBacks.empty()){
		fillCallBacks();
	}
	std::list<Sequence::method>::iterator iter = callBacks.begin();
	method fun;
	while(iter != callBacks.end()){
		fun = *iter;
		(this->*fun)();
		iter++;
	}
}

eeros::sequencer::Sequence* eeros::sequencer::Sequence::getSequence(std::string name){
	std::list<Sequence*>::iterator iter = eeros::sequencer::Sequence::allSequences.begin();
		while(iter != eeros::sequencer::Sequence::allSequences.end()){
			if((*iter)->getName().compare(name) == 0){
				return *iter;
			}
			iter++;
		}
		//throw new SequenceException();
		return 0;
}