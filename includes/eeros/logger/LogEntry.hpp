#ifndef ORG_EEROS_LOGGER_LOGENTRY_HPP_
#define ORG_EEROS_LOGGER_LOGENTRY_HPP_

#include <eeros/logger/LogWriter.hpp>
#include <memory>
#include <iostream>

namespace eeros {
namespace logger {

/**
 * LogEntries are used by the \ref Logger to create log messages. The can be 
 * daisy chained using the insertion operator.
 * 
 * @since v0.6
 */

class LogEntry {
public:
  /**
   * Creates a LogEntry which is inserted into a given \ref LogWriter.
   * @see Logger
   * 
   * @param writer - LogWriter
   * @param level - LogLevel
   * @param category - category
   */
  LogEntry(std::shared_ptr<LogWriter> writer, LogLevel level, unsigned category = 0) 
      : w(std::move(writer)) {
    enable = (level <= w->visible_level);
    if(enable) w->begin(os, level, category);
  }
  
  /** 
   * Copy constructor, default initializes the class members.
   */
  LogEntry(const LogEntry&);
  
  /**
   * Ends inserting into \ref LogWriter.
   */
  virtual ~LogEntry() {
    if(enable) w->end(os);
  }

  /**
   * Overloading the insertion operator for values of type T.
   * 
   * @tparam T - value type
   * @param value - value
   * @return LogEntry
   */
  template <typename T>
  LogEntry& operator<<(const T& value) {
    os << value;
    return *this;
  }
  
  /**
   * Overloading the insertion operator for rvalues of type T.
   * 
   * @tparam T - value type
   * @param value - rvalue
   * @return LogEntry
   */
  template <typename T> 
  LogEntry& operator<<(T&& value) {
    os << std::forward<T>(value);
    return *this;
  }
  
  /**
   * Overloading the insertion operator for functions.
   * 
   * @param f - function
   * @return LogEntry
   */
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
