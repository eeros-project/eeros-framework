#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceException.hpp>

eeros::sequencer::Sequence::Sequence(std::string name, Sequencer& caller)
	: sequenceName(name),
      callerThread(caller){
		if(&callerThread != eeros::sequencer::Sequencer::getMainSequencer()){
			eeros::sequencer::Sequencer::getMainSequencer()->addSubSequencer(&caller);
		}
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