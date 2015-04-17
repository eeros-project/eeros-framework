#include <eeros/control/TransitionBlock.hpp>

#include <iostream>

namespace {
	class TransitionTestBlock : public eeros::control::TransitionBlock {
	public:
		TransitionTestBlock() : countA(0), countB(0) { }
		
		virtual void runA() {
			countA++;
		}
		
		virtual void runB() {
			countB++;
		}
		
		int countA;
		int countB;
	};
}

int main(int argc, char* argv[]) {
	TransitionTestBlock block;
	
	eeros::Runnable *a = block.getRunnableA();
	eeros::Runnable *b = block.getRunnableB();
	
	if (block.countA != 0) {
		std::cerr << "countA should be 0" << std::endl;
		return -1;
	}
	if (block.countB != 0) {
		std::cerr << "countB should be 0" << std::endl;
		return -2;
	}
	
	a->run();
	
	if (block.countA != 1) {
		std::cerr << "countA should be 1" << std::endl;
		return -3;
	}
	if (block.countB != 0) {
		std::cerr << "countB should be 0" << std::endl;
		return -4;
	}
	
	b->run();
	
	if (block.countA != 1) {
		std::cerr << "countA should be 1" << std::endl;
		return -5;
	}
	if (block.countB != 1) {
		std::cerr << "countB should be 1" << std::endl;
		return -6;
	}
	
	return 0;
}
