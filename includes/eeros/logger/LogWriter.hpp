#ifndef ORG_EEROS_LOGGER_LOGWRITER_HPP_
#define ORG_EEROS_LOGGER_LOGWRITER_HPP_

#include <eeros/logger/Writer.hpp>

namespace eeros {
namespace logger {

enum class LogLevel {FATAL, ERROR, WARN, INFO, TRACE};

class LogWriter : public Writer {
 public:
  LogWriter() : visible_level(LogLevel::INFO) { }
  virtual ~LogWriter() { }
  virtual void begin(std::ostringstream& os, LogLevel level, unsigned category) = 0;	
  virtual void end(std::ostringstream& os) = 0;
  virtual void endl(std::ostringstream& os) = 0;
  LogLevel visible_level;
};

void endl(LogWriter& w);
std::ostringstream& operator<<(std::ostringstream& os, void (*f)(LogWriter&));

}
}

#endif /* ORG_EEROS_LOGGER_LOGWRITER_HPP_ */
