#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <string>
#include <map>
#include <functional>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <eeros/core/Runnable.hpp>
#include <eeros/sequencer/SequenceException.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

namespace eeros {
	namespace sequencer {

		// Forward declarations
		class Sequencer;

		enum SequenceState { kSequenceRunning, kSequenceFinished, kSequenceNotStarted, kSequenceException };
		

		class Sequence {
			
			friend class eeros::sequencer::Sequencer;
			
		public:
			Sequence(std::string name, Sequencer* sequencer = nullptr);
			
			virtual std::string getName();
			virtual int getState();
			
			virtual void reset();
			
			virtual void run();
			
		protected:
			void call(Sequence* sequence);
			void call(std::string sequenceName);
			void start(Sequence* sequence);
			void start(std::string sequenceName);
			virtual void init();
			virtual bool checkPreCondition();
			virtual bool checkPostCondition();
			virtual void exit();
			
			virtual void yield();
			
			virtual void addStep(std::function<void(void)> action);
			
			eeros::logger::Logger<eeros::logger::LogWriter> log;
			
		private:
			virtual void setSequencer(Sequencer* sequencer);

			Sequencer* sequencer;
			std::string name;
			SequenceState state;
			uint32_t currentStep;
			std::vector<std::function<void(void)>> steps;
			uint32_t exceptionRetryCounter;
			
		}; // class Sequence
	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
