#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <string>
#include <map>
#include <functional>
#include <vector>

#include <eeros/core/Runnable.hpp>
#include <eeros/sequencer/SequenceException.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

namespace eeros {
	namespace sequencer {

		enum SequenceState { kSequenceRunning, kSequenceFinished, kSequenceNotStarted, kSequenceException };
		

		class Sequence : public Runnable {
		
		public:
			Sequence(std::string name);
			virtual ~Sequence();
			
			virtual std::string getName();
			
			virtual int getState();
			
			virtual void run();
			
			virtual void init();
			
			virtual bool checkPreCondition();
			
			virtual bool checkPostCondition();
			
			virtual void exit();
			
			virtual void reset();
			
			void call(Sequence* sequence);
			
			template < typename T, typename ... Targs >
			void call(T& sequence, Targs ... args) {
				sequence.run(args...);
			}
			
			void start(Sequence* sequence);
			
			static Sequence* getSequence(std::string name);
			
		protected:
			virtual void addStep(std::function<void(void)> action);
			SequenceState state;
			uint32_t currentStep;
			std::string name;
			std::vector<std::function<void(void)>> actionList;
			uint32_t exceptionRetryCounter;
			eeros::logger::Logger<eeros::logger::LogWriter> log;
			
		private:
			static std::map<std::string, Sequence*> allSequences;

		}; // class Sequence

	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
