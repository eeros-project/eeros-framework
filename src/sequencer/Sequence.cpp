#include <eeros/sequencer/Sequence.hpp>

eeros::sequencer::Sequence::Sequence(std::string name, TimeDomain* ptimeDomain):
Executor(0),
sequenceName(name),
timeDomain(ptimeDomain){
}

void eeros::sequencer::Sequence::addSubSequence(Sequence* seq){
	subSequences.push_back(seq);
}

void eeros::sequencer::Sequence::deleteAllSubSequences(){
	std::list<Sequence*>::iterator iter = subSequences.begin();
	Sequence* sequence = 0;
	while(iter != subSequences.end()){
		sequence = *iter;
		delete sequence;
		iter++;
	}
	subSequences.clear();
}

void eeros::sequencer::Sequence::run(){
	run_state();
}

void eeros::sequencer::Sequence::next(method step){
	this->fun = step;
}
void eeros::sequencer::Sequence::run_state(){
	(this->*fun)();
}