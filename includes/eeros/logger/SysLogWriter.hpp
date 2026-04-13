#ifndef ORG_EEROS_LOGGER_SYSLOGWRITER_HPP_
#define ORG_EEROS_LOGGER_SYSLOGWRITER_HPP_

#include <eeros/logger/LogWriter.hpp>
#include <string>
#include <sstream>

namespace eeros::logger {

/**
 * A SysLogWriter sends log messages to the POSIX syslog facility.
 *
 * SysLogWriter opens a syslog connection on construction (via openlog()) and
 * closes it on destruction (via closelog()). Each completed log entry is
 * submitted as a single syslog message whose priority is derived from the
 * eeros \ref LogLevel:
 *
 * @since v0.6 (corrected v1.4)
 */
class SysLogWriter : public LogWriter {
 public:
  /**
   * Maximum length of the syslog identifier string including the null
   * terminator. Names longer than this are silently truncated.
   */
  static constexpr std::size_t MAX_NAME = 64;

  /**
   * Opens a syslog connection under the given application identifier.
   * The connection is opened with LOG_PID | LOG_CONS so that the process ID
   * is included in every entry and messages fall back to the console if
   * syslog is unavailable.
   *
   * @param name - application identifier shown in syslog entries (e.g. "robot")
   */
  explicit SysLogWriter(const std::string& name);
  /**
   * Closes the syslog connection.
   */
  ~SysLogWriter() override;

  /**
   * SysLogWriter is not copyable. openlog() ties the connection to a
   * specific pointer; copying would require a new openlog() call, which
   * is better expressed as constructing a new SysLogWriter explicitly.
   */
  SysLogWriter(const SysLogWriter&)            = delete;
  SysLogWriter& operator=(const SysLogWriter&) = delete;

 protected:
  void begin(std::ostringstream& os, LogLevel level, unsigned category) override;
  void end(std::ostringstream& os) override;
  void endl(std::ostringstream& os) override;

 private:
  char logName[MAX_NAME];
  LogLevel currentLevel = LogLevel::TRACE;  // set in begin(), read in end()
  static int toSyslogPriority(LogLevel level);
};

}

#endif /* ORG_EEROS_LOGGER_SYSLOGWRITER_HPP_ */
