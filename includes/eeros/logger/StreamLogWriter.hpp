#ifndef ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_
#define ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_

#include <eeros/logger/LogWriter.hpp>
#include <fstream>

namespace eeros {
namespace logger {

/**
 * A StreamLogWriter allows to send log messages to std::ostream and into log files. 
 * 
 * @since v0.6
 */

class StreamLogWriter : public LogWriter {
 public:
   
  /**
   * Creates a StreamLogWriter sending its messages to a std::ostream such as std::cout.
   * 
   * @param out - std::ostream
   */
  StreamLogWriter(std::ostream& out);
  
  /**
   * Creates a StreamLogWriter sending its messages to a std::ostream such as std::cout
   * and a log file. The file name is appended with the current time and date.
   * 
   * @param out - std::ostream
   * @param logFile - log file name
   */
  StreamLogWriter(std::ostream& out, std::string logFile);
  
  /** 
   * Destructor, closes file if necessary.
   */
  ~StreamLogWriter();

 private:  
  virtual void show(LogLevel level = LogLevel::TRACE);	
  virtual void begin(std::ostringstream& os, LogLevel level, unsigned category);
  virtual void end(std::ostringstream& os);
  virtual void endl(std::ostringstream& os);

  std::ostream& out;
  std::ofstream fileOut;
  bool colored;
};
    
}
}

#endif /* ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_ */
