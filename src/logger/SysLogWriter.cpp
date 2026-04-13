#include <eeros/logger/SysLogWriter.hpp>
#include <syslog.h>
#include <cstring>

namespace eeros::logger {

int SysLogWriter::toSyslogPriority(LogLevel level) {
  switch (level) {
    case LogLevel::FATAL: return LOG_CRIT;
    case LogLevel::ERROR: return LOG_ERR;
    case LogLevel::WARN:  return LOG_WARNING;
    case LogLevel::INFO:  return LOG_INFO;
    case LogLevel::TRACE: return LOG_DEBUG;
    default:              return LOG_DEBUG;
  }
}

SysLogWriter::SysLogWriter(const std::string& name) {
  std::strncpy(logName, name.c_str(), MAX_NAME - 1);
  logName[MAX_NAME - 1] = '\0';
  openlog(logName, LOG_PID | LOG_CONS, LOG_USER);
  syslog(LOG_INFO, "SysLogWriter opened for '%s'", logName);
}

SysLogWriter::~SysLogWriter() { closelog(); }

void SysLogWriter::begin(std::ostringstream& os, LogLevel level, unsigned category) {
  currentLevel = level;
  os.str("");
  os.clear();
}

void SysLogWriter::end(std::ostringstream& os) {
  // visible_level is the protected field inherited from LogWriter.
  // Logger already checks this before calling begin(), but SysLogWriter
  // guards here too in case it is used standalone.
  if (currentLevel <= visible_level)
    syslog(toSyslogPriority(currentLevel), "%s", os.str().c_str());
  os.str("");
  os.clear();
}

void SysLogWriter::endl(std::ostringstream& os) {
  // syslog is line-oriented; treat a logical newline the same as end().
  end(os);
}

}