#include <eeros/control/TimeDomain.hpp>

using namespace eeros::control;

TimeDomain::TimeDomain(std::string name, double period, double delay, bool realtime) : name(name), PeriodicThread(period, delay, realtime) {
	
}

void TimeDomain::run() {
	for(auto block : blocks) {
		block->run();
	}
}

void TimeDomain::addBlock(Block* block) {
	blocks.push_back(block);
}

void TimeDomain::sortBlocks() {
	// TODO
}
