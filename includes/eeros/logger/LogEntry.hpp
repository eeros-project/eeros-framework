#ifndef ORG_EEROS_LOGGER_LOGENTRY_HPP_
#define ORG_EEROS_LOGGER_LOGENTRY_HPP_

#include <eeros/logger/LogWriter.hpp>
#include <memory>
#include <iostream>
namespace eeros {
namespace logger {

class LogEntry {
 public:
  LogEntry(std::shared_ptr<LogWriter> writer, LogLevel level, unsigned category = 0) 
      : w(std::move(writer)) {
    enable = (level <= w->visible_level);
    if(enable) w->begin(os, level, category);
  }
  
  /**
   * Copy constructor
   */
  LogEntry(const LogEntry&);
  
  virtual ~LogEntry() {
    if(enable) w->end(os);
  }

  template <typename T>
  LogEntry& operator<<(const T& value) {
    os << value;
    return *this;
  }
  
  template <typename T> 
  LogEntry& operator<<(T&& value) {
    os << std::forward<T>(value);
    return *this;
  }
  
  LogEntry& operator<<(void (*f)(LogWriter&) ) {
    w->endl(os);
    return *this;
  }
  
 private:
  std::shared_ptr<LogWriter> w;
  std::ostringstream os;
  bool enable;  // every LogEntry must decide itself, if it gets logged
};

}
}

#endif /* ORG_EEROS_LOGGER_LOGENTRY_HPP_ */
