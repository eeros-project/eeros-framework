#ifndef ORG_EEROS_CORE_THREAD_HPP_
#define ORG_EEROS_CORE_THREAD_HPP_

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#include <thread>
#include <functional>
#include <string>

namespace eeros {

/**
 * This class supports non realtime and realtime threads. 
 * It can be extended to enable a subclass to run in its own thread.
 * It is used for blocks in the control system, which must run in its own thread.
 * 
 * @since v0.4
 */

class Thread {
 public:
  /**
   * Constructs a thread instance and assigns it a chosen priority. This uses 
   * root privileges. If no root privileges are delivered, the thread will use 
   * default priority.
   * If the default priority of 20 chosen, no priority assigment is made and the
   * thread will start to run with default priority. 
   * 
   * @param priority - priority of the thread
   */
  Thread(int priority = 20);
  
  /**
   * Destructor
   */
  virtual ~Thread();
  
  /** 
   * Returns the thread id.
   * 
   * @return - thread id
   */
  virtual std::string getId() const;
  
  /** 
   * Waits for the thread to finish
   */
  virtual void join();
  
 protected:
  Thread(std::function<void ()> t);
  
  virtual void run();
  
  logger::Logger log;
  std::thread t;
};

}

#endif // ORG_EEROS_CORE_THREAD_HPP_
