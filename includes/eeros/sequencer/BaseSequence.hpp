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
 * This is the base class for all \ref Sequence and \ref Step.
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
  
  /**
   * Sets the name of the sequence.
   * 
   * @param name - name of the sequence
   */
  void setName(std::string name);
  
  /**
   * Returns the name of the sequence.
   * 
   * @return - name of the sequence
   */
  std::string getName() const;
  
  /** 
   * Every sequence gets a id upon creation.
   * 
   * @param id - id of the sequence
   */
  void setId(int id);
  
  /**
   * Returns the id of the sequence.
   * 
   * @return - id of the sequence
   */
  int getId() const;
  
  /**
   * The function \ref checkExitCondition() periodically checks for the exit 
   * condition to become true. In between checks the thread will wait for 
   * polling time in ms.
   * 
   * @param timeInMilliseconds - polling time in ms
   */
  void setPollingTime(int timeInMilliseconds);
  
  /**
   * Adds a \ref Monitor to this sequence or step.
   * 
   * @param monitor - Monitor
   */
  void addMonitor(Monitor* monitor);
  
  /**
   * Sets the timeout of the timeout monitor. Each sequence owns such a timeout 
   * \ref Monitor.
   * 
   * @param timeoutInSec - timeout time in s
   */
  void setTimeoutTime(double timeoutInSec);
  
  /**
   * Reset the timeout of the timeout monitor. Each sequence owns such a timeout 
   * \ref Monitor.
   */
  void resetTimeout();

  /**
   * Sets the behavior of the timeout monitor. Each sequence owns such a timeout 
   * \ref Monitor. The behavior can be one of the following: \ref SequenceProp.
   * 
   * @param behavior - SequenceProp
   */
  void setTimeoutBehavior(SequenceProp behavior);	

  /**
   * Sets an exception sequence for the timeout monitor. Each sequence owns such a timeout 
   * \ref Monitor. The exception sequence will be executed when the monitor fires.
   * 
   * @param sequence - exception sequence
   */
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