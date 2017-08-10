#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <eeros/sequencer/BaseSequence.hpp>
#include <condition_variable>
#include <thread>

namespace eeros {
	namespace sequencer {
		
		class Sequencer;
		
		class Sequence : public BaseSequence {
		public:
			Sequence(std::string name, Sequencer& seq, BaseSequence* caller);
			Sequence(std::string name, Sequencer& Sseq);	//only for mainSequence
			virtual ~Sequence();
			
 			virtual int operator() () {return start();}	// this operator can be overloaded in the derived sequence
			virtual int action() = 0;		// this function has to be implemented in the derived sequence
			int start();
			void join() {thread->join();}
			void setBlocking() {blocking = true;}
			void setNonBlocking() {blocking = false;}
		private:
			std::mutex m;
			std::condition_variable cv;
			std::thread* thread;
			void run();
		};
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
