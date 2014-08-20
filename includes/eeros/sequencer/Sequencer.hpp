#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_

#include <string>
#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <eeros/core/Thread.hpp>

namespace eeros {
	namespace sequencer {
		
		namespace state {
			enum type { executing, waiting, terminating, terminated, idle };
		}
		
		namespace mode {
			enum type { automatic, stepping };
		}
		
		// Forward declarations
		template < typename Treturn, typename ... Targs >
		class Sequence;
		
		class Sequencer : public Thread {
		public:
			
			Sequencer();
			
			virtual bool start();
			virtual bool start(Sequence<void>* sequence);
			virtual bool start(unsigned int cmdSequenceIndex);
			virtual void shutdown();
			
			virtual void stepMode(bool on = true);
			virtual void toggleMode();
			
			virtual std::string getName() const;
			virtual state::type getState() const;
			virtual mode::type getMode() const;
		
			virtual void yield();
			virtual void proceed();
			virtual void abort();
			
			virtual void addCmdSequence(Sequence<void>* seq);
			virtual const std::vector<Sequence<void>*>& getListOfCmdSequences();
			
		protected:
			virtual void run();
			
		private:
			std::vector<Sequence<void>*> cmdSequences;
			std::atomic<Sequence<void>*> currentSequence;
			unsigned int id;
			std::atomic<state::type> state;
			std::atomic<mode::type> mode;
			std::atomic<bool> abortSequence;
			std::mutex mtx;
			std::condition_variable cv;
			bool go;
			
			static int instanceCounter;
			
		}; // class Sequencer
	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
