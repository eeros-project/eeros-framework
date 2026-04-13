#ifndef ORG_EEROS_LOGGER_LOGGER_HPP_
#define ORG_EEROS_LOGGER_LOGGER_HPP_

#include <eeros/logger/LogEntry.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <sstream>
#include <string>
#include <memory>

namespace eeros::logger {

/**
 * A logger allows to log formatted output onto various streams. 
 * Different log levels \ref LogLevel show the urgency of a log message.
 * It is further possible to block messages of a certain level below 
 * a preset log level.
 * 
 * Multiple output sinks (e.g. StreamLogWriter + SysLogWriter) can be
 * attached with \ref addWriter() — all share the same Logger interface
 * with no changes to call sites.
 *
 * ### Design notes
 *
 * - All Logger instances obtained from \ref getLogger() share the same
 *   underlying \ref LogWriter. Changing the writer via \ref setDefaultWriter()
 *   or \ref addWriter() affects all subsequent getLogger() calls.
 * - \ref show() sets the visible level on the shared writer. When the writer
 *   is a \ref MultiLogWriter, the level is propagated to all child writers.
 * - \ref addWriter() and \ref show() are implemented in Logger.cpp (not
 *   inline) to avoid a circular include dependency on MultiLogWriter.hpp.
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
   * Sets the default so that all loggers created by \ref getLogger()
   * write to a \ref StreamLogWriter targeting a std::ostream.
   *
   * @param os - output stream (e.g. std::cout)
   */
  static void setDefaultStreamLogger(std::ostream& os) {
    log = makeLogger<StreamLogWriter>(os);
  }

  /**
   * Sets the default so that all loggers created by \ref getLogger()
   * write to a \ref StreamLogWriter targeting a std::ostream and a log file.
   * The file name is appended with the current date and time.
   * 
   * @param os - output stream to which the StreamLogWriter will write
   * @param logFile - base log file name
   */
  static void setDefaultStreamLogger(std::ostream& os, std::string logFile) {
    log = makeLogger<StreamLogWriter>(os, logFile);
  }
  
  /**
   * Replaces the shared writer with any concrete LogWriter (or a
   * MultiLogWriter built externally). Must be called before the first
   * \ref getLogger().
   *
   * @param writer - fully constructed LogWriter
   */
  static void setDefaultWriter(std::shared_ptr<LogWriter> writer) {
    log.w = std::move(writer);
  }

  /**
   * Attaches an additional writer so every logger writes to both the
   * existing writer and the supplied one simultaneously. 
   * Implemented in Logger.cpp to avoid a circular dependency on MultiLogWriter.hpp.
   *
   * If the current writer is already a \ref MultiLogWriter the new
   * writer is appended to it directly; otherwise both are wrapped in a
   * new MultiLogWriter automatically. Safe to call multiple times.
   *
   * Example — stream + syslog:
   * @code
   *   Logger::setDefaultStreamLogger(std::cout);
   *   Logger::addWriter(std::make_shared<SysLogWriter>("myapp"));
   *   auto log = Logger::getLogger('A');
   *   log.show(LogLevel::WARN);   // applies to both sinks
   * @endcode
   *
   * @param writer - additional LogWriter sink to attach
   */
  static void addWriter(std::shared_ptr<LogWriter> writer);

  /**
   * Sets the visible level of this logger to a chosen level.
   * All messages with a level below this chosen level are suppressed.
   * When using \ref addWriter() / \ref MultiLogWriter the level is
   * propagated to every attached child writer.
   * Propagation to child writers of a MultiLogWriter is handled in Logger.cpp.
   * 
   * @param level - log level, messages with equal level or above get through
   */
  void show(LogLevel level = LogLevel::TRACE);

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

#endif /* ORG_EEROS_LOGGER_LOGGER_HPP_ */
