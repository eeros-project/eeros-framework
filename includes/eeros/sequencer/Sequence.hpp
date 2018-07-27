#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <eeros/sequencer/BaseSequence.hpp>
#include <condition_variable>
#include <thread>

namespace eeros {
	namespace sequencer {
		
		class Sequencer;
		
		class Sequence : public BaseSequence {
			friend class Sequencer;
		public:
			Sequence(std::string name, Sequencer& seq);	// only for mainSequence
			Sequence(std::string name, Sequencer& seq, BaseSequence* caller, bool blocking);
			virtual ~Sequence();
			
 			virtual int operator() () {return start();}	// this operator can be overloaded in the derived sequence
			virtual int action() = 0;		// this function has to be implemented in the derived sequence
			int start();
			/**
			* Waits for this sequence to finish current run. 
			* The sequence could be restarted later on.
			*/
			void wait();
			/**
			* Waits for this sequence to finish current run. 
			* As soon as this happens, its associated thread will be terminated and ceases to exist.
			*/
			void waitAndTerminate();
			
		private:
			bool running = true, go = false, done = true;
			std::thread* t;
			void run();
		};
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
