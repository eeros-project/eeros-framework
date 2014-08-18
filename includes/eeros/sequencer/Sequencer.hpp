#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_

#include <string>
#include <atomic>
#include <vector>
#include <eeros/core/Thread.hpp>
#include <mutex>
#include <condition_variable>
//#include <eeros/sequencer/Sequence.hpp>

namespace eeros {
	namespace sequencer {
		
		namespace state {
			enum type { executing, waiting, terminating, terminated, idle };
		}
		
		namespace mode {
			enum type { automatic, stepping };
		}
		
		template < typename Treturn, typename ... Targs >
		class Sequence;
		
		class Sequencer : public Thread {
		public:
			
			Sequencer();
			
			virtual void start();
			virtual void start(Sequence<void>* sequence);
			virtual void shutdown();
			
			virtual void stepMode(bool on = true);
			virtual void toggleMode();
			
			virtual std::string getName() const;
			virtual state::type getState() const;
			virtual mode::type getMode() const;
		
			virtual void yield();
			virtual void proceed();
			virtual void abort();
			
		protected:
			virtual void run();
			
		private:
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
