#include <eeros/control/TimeDomain.hpp>

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceException.hpp>

eeros::sequencer::Sequencer* eeros::sequencer::Sequencer::mainSequencer = 0;

eeros::sequencer::Sequencer* eeros::sequencer::Sequencer::getMainSequencer(){
	return mainSequencer;
}

eeros::sequencer::Sequencer::Sequencer(std::string name)
	: Executor(0),
      sequenceName(name) {
		  if(!mainSequencer){
			  eeros::sequencer::Sequencer::mainSequencer = this;
		  }
}

eeros::sequencer::Sequencer::~Sequencer(){
}

void eeros::sequencer::Sequencer::addSubSequencer(Sequencer* seq){
	eeros::sequencer::Sequencer::getMainSequencer()->subSequencers.push_back(seq);
}

void eeros::sequencer::Sequencer::deleteAllSubSequencers(){
	std::list<Sequencer*>::iterator iter = subSequencers.begin();
	Sequencer* sequencer;
	while(iter != subSequencers.end()){
		sequencer = *iter;
		delete sequencer;
		iter++;
	}
	subSequencers.clear();
}

void eeros::sequencer::Sequencer::addTimeDomain(TimeDomain* tDomain){
	eeros::sequencer::Sequencer::getMainSequencer()->timeDomains.push_back(tDomain);
}

void eeros::sequencer::Sequencer::deleteAllTimeDomains(){
	std::list<TimeDomain*>::iterator iter = timeDomains.begin();
	TimeDomain* tDomain = 0;
	while(iter != timeDomains.end()){
		tDomain = *iter;
		delete tDomain;
		iter++;
	}
	timeDomains.clear();
}

std::string eeros::sequencer::Sequencer::getName(){
	return sequenceName;
}

eeros::sequencer::Sequencer* eeros::sequencer::Sequencer::findSequencer(std::string name){
	std::list<Sequencer*>::iterator iter = subSequencers.begin();
		while(iter != subSequencers.end()){
			if((*iter)->getName().compare(name) == 0){
				return *iter;
			}
			iter++;
		}
		//throw new SequenceException();
		return 0;
}

void eeros::sequencer::Sequencer::deleteSequencer(std::string name){
	std::list<Sequencer*>::iterator iter = subSequencers.begin();
		while(iter != subSequencers.end()){
			if((*iter)->getName().compare(name) == 0){
				subSequencers.erase(iter);
				return;
			}
			iter++;
		}
}