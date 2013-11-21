#include <eeros/sequencer/Sequencer.hpp>

#include <eeros/control/TimeDomain.hpp>

using namespace eeros::sequencer;
using namespace eeros::control;

Sequencer* Sequencer::mainSequencer = 0;

Sequencer* Sequencer::getMainSequencer(){
	return mainSequencer;
}

Sequencer::Sequencer(std::string name)
	: Executor(0),
      sequencerName(name) {
	if(!mainSequencer){
		  Sequencer::mainSequencer = this;
	}
}

Sequencer::~Sequencer(){	
	if(Sequencer::mainSequencer == this){
		  Sequencer::mainSequencer = 0;
	}else{
		deleteSequencer(sequencerName);
	}
}

void Sequencer::addSubSequencer(Sequencer* seq){
	Sequencer::getMainSequencer()->subSequencers.push_back(seq);
}

void Sequencer::deleteAllSubSequencers(){
	std::list<Sequencer*>::iterator iter = subSequencers.begin();
	Sequencer* sequencer;
	while(iter != subSequencers.end()){
		sequencer = *iter;
		delete sequencer;
		iter++;
	}
	subSequencers.clear();
}

void Sequencer::addTimeDomain(TimeDomain* tDomain){
	Sequencer::getMainSequencer()->timeDomains.push_back(tDomain);
}

void Sequencer::deleteAllTimeDomains(){
	std::list<TimeDomain*>::iterator iter = timeDomains.begin();
	TimeDomain* tDomain = 0;
	while(iter != timeDomains.end()){
		tDomain = *iter;
		delete tDomain;
		iter++;
	}
	timeDomains.clear();
}

std::string Sequencer::getName(){
	return sequencerName;
}

Sequencer* Sequencer::findSequencer(std::string name) {
	std::list<Sequencer*>::iterator iter = subSequencers.begin();
		while(iter != subSequencers.end()){
			if((*iter)->getName().compare(name) == 0){
				return *iter;
			}
			iter++;
		}
		throw "no SubSequencer found";
		return 0;
}

void Sequencer::deleteSequencer(std::string name){
	std::list<Sequencer*>::iterator iter = subSequencers.begin();
	while(iter != subSequencers.end()){
		if((*iter)->getName().compare(name) == 0){
			subSequencers.erase(iter);
			return;
		}
		iter++;
	}
}