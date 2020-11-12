#ifndef ORG_EEROS_CORE_THREAD_HPP_
#define ORG_EEROS_CORE_THREAD_HPP_

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#include <thread>
#include <functional>
#include <string>

namespace eeros {

class Thread {
 public:
  Thread(int priority);
  virtual ~Thread();
  
  virtual std::string getId() const;
  virtual void join();
  
 protected:
  Thread(std::function<void ()> t);
  
  virtual void run();
  
  logger::Logger log;
  std::thread t;
// 		int priority;
};
};

#endif // ORG_EEROS_CORE_THREAD_HPP_
