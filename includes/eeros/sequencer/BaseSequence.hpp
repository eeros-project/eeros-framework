#ifndef ORG_EEROS_SEQUENCER_BASESEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_BASESEQUENCE_HPP_

#include <eeros/logger/Logger.hpp>
#include <eeros/sequencer/ConditionTimeout.hpp>
#include <eeros/sequencer/ConditionAbort.hpp>
#include <eeros/sequencer/Monitor.hpp>
#include <vector>


namespace eeros {
	namespace sequencer {
		
		using namespace eeros::logger;
		class Sequencer;
		
		enum class SequenceState {
			idle,		// upon creation
			running,	// active and running
			paused,		// not used
			aborting,	// to be stopped, due to restarting of caller sequence
			aborted,	// stopped, due to restarting of caller sequence
			terminated,	// precondition failed, exitcondition successful
			restarting,	// repeat 
		};

		class BaseSequence {	
			friend class Monitor;
			friend class Sequencer;
			friend class Sequence;
		public:
			BaseSequence(Sequencer& seq, BaseSequence* caller, bool blocking);
			BaseSequence(BaseSequence* caller, bool blocking);
			virtual ~BaseSequence();
			
			virtual int start() = 0;
			virtual bool checkPreCondition();
			virtual bool checkExitCondition();
			
			void setName(std::string name);
			std::string getName() const;
			void setId(int id);
			int getId() const;		// steps allways have id=-99
			
			BaseSequence* getCallerSequence();
			std::vector<BaseSequence*> getCallerStack() const;
			
			SequenceState getRunningState() const;
			void setPollingTime(int timeInMilliseconds);
			
			// Monitors
			std::vector<Monitor*> monitors;
			void addMonitor(Monitor* monitor);
			std::vector<Monitor*> getMonitors() const;
			
			// Timeout
			void setTimeoutTime(double timeoutInSec);		// in seconds. For this sequence
			void resetTimeout();
			void setTimeoutBehavior(SequenceProp behavior);	
			void setTimeoutExceptionSequence(BaseSequence& sequence);
			
			// Abort
			void resetAbort();
			
		protected:
			virtual int action();		// handles different checks like preconditions
			virtual int operator() () = 0;	// has to be implemented in derived class	
		
			std::string name;			
			Sequencer& seq;			// reference to sequencer
			BaseSequence* caller;		// calling sequence
			bool blocking;			// standard run mode
			bool exceptionIsActive = false;	// one of its monitors fired
			bool inExcProcessing = false;	// this sequence already started an exception sequence of one of its monitors
			SequenceState state;	
			Logger log;
			
		private:
			void checkMonitorsOfBlockedCallers();
			void checkMonitorsOfThisSequence();
			void checkMonitor(Monitor* monitor);
			void setActiveException(Monitor* activeMonitor);
			void clearActiveException();
			void checkActiveException();
			
			int id;
			std::vector<BaseSequence*> callerStack;		// vector with all caller sequences. Latest element is latest caller
			std::vector<BaseSequence*> callerStackBlocking;	// vector with all sequences, which are blocked by this sequence. Element[0] is the oldest blocked caller
			bool callerStackCreated = false;
			Monitor monitorTimeout;
			ConditionTimeout conditionTimeout;
			Monitor monitorAbort;
			ConditionAbort conditionAbort;
			int pollingTime;		//in milliseconds for checkExitCondition monitors)
			Monitor* activeException;
		};
		
		/********** Print functions **********/
		std::ostream& operator<<(std::ostream& os, SequenceState state);

	};	// namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_BASESEQUENCE_HPP_