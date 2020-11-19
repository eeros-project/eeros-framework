#ifndef ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_

#include <eeros/logger/Logger.hpp>
#include <future>

namespace eeros {
namespace sequencer {
    
void sigPipeHandler(int signum);

/**
 * The \ref Sequencer can be remotely controlled by this simple user interface.
 * This utility opens a network connection and controls then running of the registered 
 * sequences in the sequencer. Sequences can be stopped, single stepped or continued.
 * 
 * @since v1.0
 */

class SequencerUI {
 public:
   
  /**
   * Creates and handles a socket connection to a user interface for the sequencer. 
   * Runs in its own thread.
   */
  SequencerUI();
  
  /**
   * Destructor, stops the thread.
   */
  virtual ~SequencerUI();
   
 private:
  virtual void run();
  bool running;
  uint16_t port;
  double period;
  struct hostent *server;
  int sockfd;
  int newsockfd;
  std::future<void> fut;
  logger::Logger log;  
};

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_