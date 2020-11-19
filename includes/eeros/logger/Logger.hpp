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

/**
 * A logger allows to log formatted output onto various streams. 
 * Different log levels \ref LogLevel show the urgency of a log message.
 * It is further possible to block messages of a certain level below 
 * a preset log level.
 * 
 * @since v0.6
 */

class Logger {
 public:
   
  /**
   * Generates a log message with log level FATAL. The return value is a \ref LogEntry.
   * This value can be chained with further LogEntries and will be written onto the 
   * given \ref LogWriter of this logger.
   * 
   * @return - LogEntry
   */
  LogEntry fatal() { return LogEntry(w, LogLevel::FATAL, category); }

  /**
   * Generates a log message with log level ERROR. The return value is a \ref LogEntry.
   * This value can be chained with further LogEntries and will be written onto the 
   * given \ref LogWriter of this logger.
   * 
   * @return - LogEntry
   */
  LogEntry error() { return LogEntry(w, LogLevel::ERROR, category); }

  /**
   * Generates a log message with log level WARN. The return value is a \ref LogEntry.
   * This value can be chained with further LogEntries and will be written onto the 
   * given \ref LogWriter of this logger.
   * 
   * @return - LogEntry
   */
  LogEntry warn() { return LogEntry(w, LogLevel::WARN, category); }

  /**
   * Generates a log message with log level INFO. The return value is a \ref LogEntry.
   * This value can be chained with further LogEntries and will be written onto the 
   * given \ref LogWriter of this logger.
   * 
   * @return - LogEntry
   */
  LogEntry info() { return LogEntry(w, LogLevel::INFO, category); }

  /**
   * Generates a log message with log level TRACE. The return value is a \ref LogEntry.
   * This value can be chained with further LogEntries and will be written onto the 
   * given \ref LogWriter of this logger.
   * 
   * @return - LogEntry
   */
  LogEntry trace() { return LogEntry(w, LogLevel::TRACE, category); }
  
  /**
   * Returns a new logger with a chosen category. The category must
   * be a capital letter (A .. Z).
   * 
   * @param category - category of the logger, capital letter A .. Z
   * @return - Logger
   */
  static Logger getLogger(unsigned category = 0) {
    log.category = category;
    return log;
  }
 
  /**
   * Sets the default in such a way that all logger that will be created by
   * \ref getLogger() will have their \ref LogWriter set to a \ref StreamLogWriter.
   * The \ref StreamLogWriter will write to a std::ostream.
   * 
   * @param os - output stream to which the StreamLogWriter will write
   */
  static void setDefaultStreamLogger(std::ostream& os) {
    log = makeLogger<StreamLogWriter>(os);
  }
  
  /**
   * Sets the default in such a way that all logger that will be created by
   * \ref getLogger() will have their \ref LogWriter set to a \ref StreamLogWriter.
   * The \ref StreamLogWriter will write to a std::ostream and into a log file.
   * 
   * @param os - output stream to which the StreamLogWriter will write
   * @param logFile - log file name
   */
  static void setDefaultStreamLogger(std::ostream& os, std::string logFile) {
    log = makeLogger<StreamLogWriter>(os, logFile);
  }
  
  /**
   * Sets the visible level of this logger to a chosen level.
   * All messages with a level below this chosen level are suppressed.
   * 
   * @param level - log level, messages with equal level or above get through
   */
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
