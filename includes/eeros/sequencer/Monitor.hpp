#ifndef ORG_EEROS_SEQUENCER_MONITOR_HPP_
#define ORG_EEROS_SEQUENCER_MONITOR_HPP_

#include <string>

namespace eeros {
namespace sequencer {

class BaseSequence;
class Condition;

enum class SequenceProp {
  resume,				// continue (only exception sequence is called, if available)
  abort,				// abort the owner sequence or step of this monitor
  restart				// restart the owner sequence or step of this monitor
};

/**
 * A monitor checks its associated \ref condition whenever its owning \ref sequence or \ref step
 * runs. When a monitor fires it will cause its exception sequence to run (if defined). Subsequently 
 * it will return to its owning sequence. It could simply resume this sequence, which means that it will continue
 * where it was interrupted by the monitor. It could abort the rest of the sequence or it could restart the whole 
 * sequence.
 * 
 * @since v1.0
 */
class Monitor {
  friend class BaseSequence;
 public:
  /**
   * Constructs a monitor instance.
   * 
   * @param name - name of the monitor
   * @param owner - owning sequence or step
   * @param condition - the condition the monitor will check
   * @param behavior - what to do with the owning sequence, after the exception is handled
   * @param exceptionSequence - sequence that will run when the monitor fires
   */
  Monitor(std::string name, BaseSequence* owner, Condition& condition, SequenceProp behavior = SequenceProp::resume, BaseSequence* exceptionSequence = nullptr);

  /**
   * Destructor
   */
  virtual ~Monitor();

  /**
   * Registers a sequence which will run as soon as a monitor fires. 
   * This exception sequence can be any regular sequence or step. It will run while the sequence 
   * which owns the monitor will simply block during this run time.
   * 
   * @param exceptionSequence - sequence that will run when the monitor fires
   */
  void setExceptionSequence(BaseSequence& exceptionSequence);	

  /**
   * Sets the behavior of the sequence which owns the monitor. This behavior describes what this 
   * owning sequence does when the exception sequence has finished running. 
   * It could simply resume, which means that it will continue where it was interrupted by the monitor. 
   * It could abort the rest of the sequence or it could restart the whole sequence.
   * 
   * @param behavior - what to do with the owning sequence, after the exception is handled
   */
  void setBehavior(SequenceProp behavior);

  /**
   * Returns the behavior.
   * @see void setBehavior(SequenceProp behavior);
   * 
   * @return  what to do with the owning sequence, after the exception is handled
   */
  SequenceProp getBehavior() const;

  /**
   * Returns the owner.
   * 
   * @return  the sequence or step which owns this monitor
   */
  BaseSequence* getOwner() const;
  
 protected:
  bool checkCondition();
  void startExceptionSequence();
  BaseSequence* owner;	// every monitor has an owner, sequence property of monitor determines what owner does
  BaseSequence* exceptionSequence;
  Condition& condition;
  SequenceProp behavior;
  std::string name;
};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, SequenceProp prop);

} // namespace sequencer
} // namespace eeros

#endif	// ORG_EEROS_SEQUENCER_MONITOR_HPP_