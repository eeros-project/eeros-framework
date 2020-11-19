#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#include <eeros/sequencer/SequencerUI.hpp>
#include <eeros/logger/Logger.hpp>
#include <vector>
#include <atomic>
#include <memory>

namespace eeros {
namespace sequencer {

class Sequence;
class SequencerUI;

/**
 * The sequencer keeps a list of all \ref Sequence. It allows to abort all running sequences
 * and can wait for all sequences to finish running. Further it is possible to do single stepping
 * of sequences with the aid of a simple user interface.
 * 
 * @since v1.0
 */
class Sequencer {
  friend class BaseSequence;
  friend class Sequence;
  friend class SequencerUI;
  
 public:

  /**
   * Returns a sequencer instance. When first called it will initially create such an instance.
   * Subsequent calls will return this initial instance.
   * 
   * @return - sequencer instance
   */
  static Sequencer& instance();
  
  /**
   * Every registered sequence has a unique identifier. This function returns the sequence 
   * with a given identifier
   * 
   * @param id - id of a sequence
   * @return - sequence with this id
   */
  Sequence* getSequenceById(int id);

  /**
   * Every registered sequence has a unique name. This function returns the sequence 
   * with a given name
   * 
   * @param name - id of a sequence
   * @return - sequence with this name
   */
  Sequence* getSequenceByName(std::string name);

  /**
   * Returns a vector containing all registered sequences.
   * 
   * @return - vector with all registered sequences
   */
  std::vector<Sequence*> getListOfAllSequences();
  
  /**
   * Clears the list with all sequences
   */
  void clearList();
  
  /**
   * Waits for the sequencer to terminate all its running sequences
   * (either blocking or nonblocking)
   */
  void wait();

  /**
   * Aborts all running sequences. Please make sure to wait for these sequences to finish running.
   * @see void wait()
   * 
   */
  void abort();
  
  /**
   * The sequencer can be put into single stepping mode. This can be useful for
   * debugging purposes, E.g. upon entering a given sequence. 
   */
  void singleStepping();
  
  /**
   * State of the sequencer, set to true upon creation. Aborting the sequencer will 
   * set this variable to false and will abort all registered sequences.
   */
  static std::atomic<bool> running;

 private:
  Sequencer();
  void addSequence(Sequence& seq);
  void step();
  void restart();
  Sequence* mainSequence;
  std::vector<Sequence*> sequenceList;	// list of all sequences
  logger::Logger log;	
  std::atomic<bool> stepping;
  volatile std::atomic<bool> nextStep;
  SequencerUI ui;
  static int sequenceCount;
};

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
