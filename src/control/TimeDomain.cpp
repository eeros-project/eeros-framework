#include <eeros/control/TimeDomain.hpp>

using namespace eeros::control;

TimeDomain::TimeDomain(std::string name, double period, bool realtime) : name(name), PeriodicThread(period, 0, realtime, paused) {
	// nothing to do
}

void TimeDomain::run() {
	for(auto block : blocks) {
		block->run();
	}
}

void TimeDomain::addBlock(Block* block) {
	blocks.push_back(block);
}

// void TimeDomain::sortBlocks() {
// 	// TODO
// }

void TimeDomain::enable() {
	start();
}

void TimeDomain::disable() {
	pause();
}
