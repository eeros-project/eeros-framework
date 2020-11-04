#ifndef ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_

#include <eeros/core/Thread.hpp>
#include <future>

namespace eeros {
namespace sequencer {
    
void sigPipeHandler(int signum);

class SequencerUI {
 public:
  SequencerUI();
  
  virtual ~SequencerUI();
  
  virtual bool isRunning();
  
 private:
  virtual void run();
  bool running;
  uint16_t port;
  double period;
  struct hostent *server;
  int sockfd;
  int newsockfd;
  std::future<void> fut;
  eeros::logger::Logger log;  
};

} // namespace sequencer
} // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_