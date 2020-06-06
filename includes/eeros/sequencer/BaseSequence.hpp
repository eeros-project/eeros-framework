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
			
  /**
   * Aborts this sequence.
   * The sequence will have to finish its action method beforehand, then check its
   * exit condition. The sequence will not terminate until the exit condition becomes true.
   * With abort you can stop this sequence with the exit condition still being false.
   */
   void abort();
			
		protected:
			virtual int action();		// handles different checks like preconditions
			virtual int operator() () = 0;	// has to be implemented in derived class	
            // Abort
            void resetAbort();
		
			std::string name;			
			Sequencer& seq;			// reference to sequencer
			BaseSequence* caller;		// calling sequence
			bool blocking;			// standard run mode
			bool monitorFired = false;	// one of its monitors fired
			bool inExcProcessing = false;	// this sequence started an exception sequence due to one of its monitors which is still running
			SequenceState state;	
			Logger log;
			
		private:
			void checkMonitorsOfBlockedCallers();
			void checkMonitorsOfThisSequence();
			void checkMonitor(Monitor* m);	// check if monitor has fired, set state and active monitor of owner sequence, start exception sequence
			void clearActiveMonitor();	// clears any active monitor
			/**
			* Checks if this or a sequence in the call stack of this sequence has a monitor which fired. 
			* If so, two cases must be considered. 
			* Case 1: monitor of this sequence fired
			*	set sequence state of this sequence to monitor property
			* Case 2: monitor of one of the callers fired
			*	set sequence state of this sequence to monitor property 
			*   repeat setting the state of all the sequences in the call stack backwards up to the sequence
			*   which owns the monitor who fired
			*/
			void checkActiveMonitor();
			
			int id;
			std::vector<BaseSequence*> callerStack;		// vector with all caller sequences. Latest element is latest caller
			std::vector<BaseSequence*> callerStackReverse;	// vector with all sequences, which are blocked by this sequence. Element[0] is the oldest blocked caller
			bool callerStackCreated = false;
			Monitor monitorTimeout;
			ConditionTimeout conditionTimeout;
			Monitor monitorAbort;
			ConditionAbort conditionAbort;
			int pollingTime;		//in milliseconds for checkExitCondition monitors)
			Monitor* activeMonitor;	// monitor, which fired and caused exception sequence to run
		};
		
		/********** Print functions **********/
		std::ostream& operator<<(std::ostream& os, SequenceState state);

	};	// namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_BASESEQUENCE_HPP_