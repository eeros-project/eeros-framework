#include <eeros/control/TimeDomain.hpp>

using namespace eeros::control;

TimeDomain::TimeDomain(std::string name, double period, bool realtime) :
	name(name), period(period), realtime(realtime) {
	// nothing to do
}

std::string TimeDomain::getName() {
	return name;
}

double TimeDomain::getPeriod() {
	return period;
}

bool TimeDomain::getRealtime() {
	return realtime;
}

void TimeDomain::registerSafetyEvent(SafetySystem* ss, SafetyEvent* e) {
	safetySystem = ss;
	safetyEvent = e;
}

void TimeDomain::run() {
	if(!running) return;
	try {
		for(auto block : blocks) block->run();
	} catch (NotConnectedFault& e) {
		if(safetySystem != nullptr && safetyEvent != nullptr) {
			safetySystem->triggerEvent(*safetyEvent);
			safetySystem->log.error() << "Input of block not connected";
		} else throw eeros::Fault("Time domain cannot trigger safety event");
	} catch (NaNOutputFault& e) {
		if(safetySystem != nullptr && safetyEvent != nullptr) {
			safetySystem->triggerEvent(*safetyEvent);
			safetySystem->log.error() << "Try to write NaN to peripheral output";
		} else throw eeros::Fault("Time domain cannot trigger safety event");
	}
}

void TimeDomain::start() {
	running = true;
}

void TimeDomain::stop() {
	running = false;
}

void TimeDomain::addBlock(eeros::Runnable* block) {
	blocks.push_back(block);
}

void TimeDomain::addBlock(eeros::Runnable& block) {
	blocks.push_back(&block);
}

void TimeDomain::removeBlock(eeros::Runnable* block) {
	blocks.remove(block);
}

void TimeDomain::removeBlock(eeros::Runnable& block) {
	blocks.remove(&block);
}

// void TimeDomain::sortBlocks() {
// 	// TODO
// }

namespace eeros {
	namespace control {
		std::ostream& operator<<(std::ostream& os, TimeDomain& td) {
			os << "Time domain: '" << td.getName() << "'"; 
		}
	}
}