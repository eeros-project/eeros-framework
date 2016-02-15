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

void TimeDomain::run() {
	for(auto block : blocks) {
		block->run();
	}
}

void TimeDomain::addBlock(eeros::Runnable* block) {
	blocks.push_back(block);
}

// void TimeDomain::sortBlocks() {
// 	// TODO
// }
