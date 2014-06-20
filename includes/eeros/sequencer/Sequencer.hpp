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
			
			enum State { executing, waiting, terminating, terminated, idle };
			enum Mode { automatic, stepping };
			
			Sequencer(Sequence* startSequence = nullptr);
			Sequencer(Sequence& startSequence);
			
			virtual bool registerSequence(Sequence* sequence);
			virtual Sequence* getRegisteredSequence(std::string name);
			virtual bool isRegistered(const Sequence* sequence) const;
			virtual const std::map<std::string, Sequence*>& getListOfRegisteredSequences();
			virtual const Sequence* getCurrentSequence() const;
			
			virtual void start();
			virtual void start(Sequence* sequence);
			virtual void stepMode(bool on = true);
			virtual void toggleMode();
			virtual void shutdown();
			
			virtual std::string getName() const;
			virtual State getState() const;
			virtual Mode getMode() const;
		
			virtual void yield();
			virtual void proceed();
			virtual void abort();
			
		protected:
			virtual void run();
			
		private:
			unsigned int id;
			std::map<std::string, Sequence*> sequences;
			std::atomic<Sequence*> currentSequence;
			std::atomic<State> state;
			std::atomic<Mode> mode;
			std::mutex mtx;
			std::condition_variable cv;
			bool go;
			
			static int instanceCounter;
			
		}; // class Sequence
	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
