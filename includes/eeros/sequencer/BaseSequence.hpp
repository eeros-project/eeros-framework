#ifndef ORG_EEROS_SEQUENCER_BASESEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_BASESEQUENCE_HPP_

#include <eeros/logger/Logger.hpp>
#include <eeros/sequencer/ConditionTimeout.hpp>
#include <eeros/sequencer/ConditionAbort.hpp>
#include <eeros/sequencer/Monitor.hpp>
#include <vector>
#include <mutex>


namespace eeros {
namespace sequencer {

using namespace eeros::logger;
class Sequencer;

enum class SequenceState {
  idle,         // upon creation
  starting,     // started, before first run
  running,      // active and running
  paused,       // not used
  aborting,     // to be stopped, due to restarting of caller sequence
  terminated,   // precondition failed or exit condition successful
  restarting    // repeat 
};

/**
 * This is the base class for all \ref sequences and \ref steps.
 * It defines the common basic functionalities.
 * 
 * @since v1.0
 */
class BaseSequence {
  friend class Monitor;
  friend class Sequencer;
  friend class Sequence;
 public:
  /**
   * Constructs a base sequence instance.
   * 
   * @param seq - sequencer
   * @param caller - calling sequence
   * @param blocking - will be blocking if true
   */
  BaseSequence(Sequencer& seq, BaseSequence* caller, bool blocking);

  /**
   * Constructs a base sequence instance.
   * 
   * @param caller - calling sequence
   * @param blocking - will be blocking if true
   */
  BaseSequence(BaseSequence* caller, bool blocking);

  /**
   * Destructor
   */  
  virtual ~BaseSequence();
  
  /** 
   * Before a sequence or step can run, its precondition must be checked. The sequence
   * or step will run only in case that this check returns true.
   * 
   * @return - true if condition is met
   */
  virtual bool checkPreCondition();

  /** 
   * A sequence or step will execute its action and subsequenctly wait for the 
   * exit condition to become true. It will block as long as the check for the 
   * exit condition returns false.
   * 
   * @return - true if condition is met
   */
  virtual bool checkExitCondition();
  
  void setName(std::string name);
  std::string getName() const;
  /* 
   * Every sequence gets a id upon creation.
   */
  void setId(int id);
  int getId() const;		// steps allways have id=-99
  
  void setPollingTime(int timeInMilliseconds);
  
  // Monitors
  void addMonitor(Monitor* monitor);
  
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
  virtual int start() = 0;  // has to be implemented in derived class 
  virtual int action(); // has to be implemented in custom step or sequence
  virtual int operator() () = 0; // has to be implemented in derived class	
  void resetAbort();

  std::string name;
  Sequencer& seq; // reference to sequencer
  BaseSequence* caller; // calling sequence
  bool blocking;  // standard run mode
  bool inExcProcessing = false;	// this sequence started an exception sequence due to one of its monitors which is still running
  SequenceState state;	
  Logger log;
  
 private:
  void checkMonitors();
  BaseSequence* checkMonitor(Monitor* m);
  void clearActiveMonitor();	// clears any active monitor
  std::vector<Monitor*> getMonitors() const;
  
  int id;
  std::vector<Monitor*> monitors;
  bool monitorFired = false;  // one of its monitors fired
  std::vector<BaseSequence*> callerStack; // vector with all caller sequences including this sequence
  Monitor monitorTimeout;
  ConditionTimeout conditionTimeout;
  Monitor monitorAbort;
  ConditionAbort conditionAbort;
  int pollingTime;  // in milliseconds for checkExitCondition monitors)
  Monitor* activeMonitor; // monitor, which fired and causes exception sequence to run
  std::mutex mtx;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * sequence instance to an output stream.\n
 * Does not print a newline control character.
 */
std::ostream& operator<<(std::ostream& os, const SequenceState& state);

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_BASESEQUENCE_HPP_