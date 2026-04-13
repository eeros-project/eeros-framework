#ifndef ORG_EEROS_LOGGER_LOGWRITER_HPP_
#define ORG_EEROS_LOGGER_LOGWRITER_HPP_

#include <eeros/logger/Writer.hpp>

namespace eeros::logger {

/**
 * Defines the urgency levels for log messages, in descending order of severity.
 * A logger configured to show a given level will display all messages at that
 * level and above (i.e. more severe).
 */
enum class LogLevel {FATAL, ERROR, WARN, INFO, TRACE};

class Logger;
class LogEntry;
class MultiLogWriter;

/**
 * Abstract base class for all log output backends.
 *
 * A LogWriter receives log messages from \ref LogEntry and writes them to a
 * concrete destination such as a stream, a file, or the system log. Subclasses
 * implement the three-phase protocol:
 *
 *  1. \ref begin()  — called once when a new log entry starts. The writer
 *                     should format and write any prefix (timestamp, level
 *                     indicator, category) into the provided stream buffer.
 *  2. operator<<    — content is appended to the stream buffer by \ref LogEntry
 *                     directly; the writer does not participate in this phase.
 *  3. \ref end()    — called once when the entry is complete. The writer should
 *                     flush the accumulated buffer to its destination.
 *  3a. \ref endl()  — called instead of end() when a logical line break is
 *                     requested mid-entry. Line-oriented destinations (e.g.
 *                     syslog) should flush on endl() the same way as on end().
 *
 * The \ref visible_level field acts as a gate: \ref LogEntry compares the
 * incoming message level against visible_level before calling begin(), so
 * messages below the threshold are discarded without any writer involvement.
 *
 * LogWriter's interface is intentionally protected. Only \ref Logger,
 * \ref LogEntry, and \ref MultiLogWriter (all declared as friends) may call
 * begin/end/endl or read visible_level directly. User code interacts with
 * loggers exclusively through the \ref Logger API.
 *
 * @since v0.6
 */
class LogWriter : public Writer {
  friend class Logger;
  friend class LogEntry;
  friend class MultiLogWriter;

 protected:
  /**
   * Constructs a LogWriter with a default visible level of INFO.
   * Messages at INFO, WARN, ERROR, and FATAL are shown; TRACE is suppressed.
   */
  LogWriter() : visible_level(LogLevel::INFO) { }

  /**
   * Virtual destructor. Ensures correct cleanup of derived writers when
   * destroyed through a base-class pointer (e.g. inside a shared_ptr<LogWriter>).
   */
  virtual ~LogWriter() = default;

  /**
   * Called at the start of each log entry. Implementations should write any
   * per-entry prefix — such as a timestamp, log level indicator, or category
   * letter — into \p os.
   *
   * @param os       - output buffer for this log entry
   * @param level    - severity level of the incoming message
   * @param category - category identifier (typically a capital letter A–Z,
   *                   or 0 for uncategorised)
   */
  virtual void begin(std::ostringstream& os, LogLevel level, unsigned category) = 0;	

  /**
   * Called at the end of each log entry. Implementations should flush the
   * content accumulated in \p os to their destination (stream, file, syslog,
   * etc.) and then clear the buffer.
   *
   * @param os - output buffer containing the complete log entry
   */
  virtual void end(std::ostringstream& os) = 0;

  /**
   * Called when a logical newline is inserted mid-entry (via eeros::logger::endl).
   * Line-oriented destinations should flush \p os as if end() were called.
   * Stream-oriented destinations may instead append a newline character and
   * continue accumulating.
   *
   * @param os - output buffer containing the entry content up to this point
   */
  virtual void endl(std::ostringstream& os) = 0;

  /**
   * The minimum log level that this writer will output.
   * Messages whose level is numerically greater than visible_level (i.e. less
   * severe) are suppressed. Set via \ref Logger::show().
   *
   * Default: LogLevel::INFO
   */
  LogLevel visible_level;
};

/**
 * Inserts a logical newline into a log entry, causing line-oriented writers
 * to flush the current line immediately.
 *
 * @param w - the LogWriter to notify
 */
void endl(LogWriter& w);

/**
 * Overloads the stream insertion operator to allow endl (and similar
 * manipulators) to be chained into a \ref LogEntry using operator<<.
 *
 * @param os - the current log entry buffer
 * @param f  - manipulator function (e.g. eeros::logger::endl)
 * @return   - the same buffer, for further chaining
 */
std::ostringstream& operator<<(std::ostringstream& os, void (*f)(LogWriter&));

}

#endif /* ORG_EEROS_LOGGER_LOGWRITER_HPP_ */
