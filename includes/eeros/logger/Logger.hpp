#ifndef ORG_EEROS_LOGGER_LOGGER_HPP_
#define ORG_EEROS_LOGGER_LOGGER_HPP_

#include <eeros/logger/LogEntry.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <sstream>
#include <string>
#include <memory>

namespace eeros {
namespace logger {

class Logger {
 public:
  virtual ~Logger() { }
  
  LogEntry fatal() { return LogEntry(w, LogLevel::FATAL, category); }
  LogEntry error() { return LogEntry(w, LogLevel::ERROR, category); }
  LogEntry warn() { return LogEntry(w, LogLevel::WARN, category); }
  LogEntry info() { return LogEntry(w, LogLevel::INFO, category); }
  LogEntry trace() { return LogEntry(w, LogLevel::TRACE, category); }
  
  static Logger getLogger(unsigned category = 0) {
    log.category = category;
    return log;
  }
 
  static void setDefaultStreamLogger(std::ostream& os) {
    log = makeLogger<StreamLogWriter>(os);
  }
  
  static void setDefaultStreamLogger(std::ostream& os, std::string logFile) {
    log = makeLogger<StreamLogWriter>(os, logFile);
  }
  
  void show(LogLevel level = LogLevel::TRACE) {
    w->visible_level = level;
  }

 private:
  std::shared_ptr<LogWriter> w;
  unsigned category;

  Logger(std::shared_ptr<LogWriter>&& writer): w(writer) { }
  template <typename ConcreteWriter, typename ... Args>
  static Logger makeLogger(Args&& ... args) {
    return Logger(std::make_shared<ConcreteWriter>(std::forward<Args>(args)...));
  }
  static Logger log;
};

}
}

#endif /* ORG_EEROS_LOGGER_LOGGER_HPP_ */
