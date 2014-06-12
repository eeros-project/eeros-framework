#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_

#include <string>
#include <atomic>
#include <eeros/core/Thread.hpp>
#include <eeros/sequencer/Sequence.hpp>

namespace eeros {
	namespace sequencer {

		class Sequencer : public Thread {
		public:
			
			enum status { running, stepping, stopping, stopped, waiting };
			
			Sequencer(Sequence* startSequence = nullptr);
			Sequencer(Sequence& startSequence);
			
			virtual bool registerSequence(Sequence* sequence);
			virtual Sequence* getRegisteredSequence(std::string name);
			virtual bool isRegistered(const Sequence* sequence);

// 			virtual bool setStartSequence(Sequence* s);
			
			virtual void start(bool stepMode = false);
			virtual void start(Sequence* sequence, bool stepMode = false);
			virtual void stepMode(bool on);
			virtual void shutdown();
		
			virtual void yield();
			virtual void proceed();
			
		protected:
			virtual void run();
			
		private:
			int id;
			std::map<std::string, Sequence*> sequences;
			std::atomic<Sequence*> currentSequence;
			std::atomic<status> s;
			std::mutex mtx;
			std::condition_variable cv;
			bool go;
			
			static int instanceCounter;
			
		}; // class Sequence
	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
