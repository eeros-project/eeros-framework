#ifndef ORG_EEROS_SEQUENCER_WAIT_HPP_
#define ORG_EEROS_SEQUENCER_WAIT_HPP_

#include <eeros/sequencer/Step.hpp>

namespace eeros {
	namespace sequencer {

		class Wait : public Step {
		public:
			Wait(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
			virtual ~Wait() { };
			
			int operator() (double waitingTime) {this->waitingTime = waitingTime; return start();}
			int action() {time = std::chrono::steady_clock::now();}
			bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > waitingTime;}
			
			std::chrono::time_point<std::chrono::steady_clock> time;
			double waitingTime;
		};
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_WAIT_HPP_
