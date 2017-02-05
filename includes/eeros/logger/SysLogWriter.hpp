#ifndef ORG_EEROS_LOGGER_SYSLOGWRITER_HPP_
#define ORG_EEROS_LOGGER_SYSLOGWRITER_HPP_

#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/Logger.hpp>
#include <sstream>

namespace eeros {
	namespace logger {
		class SysLogWriter : public LogWriter {
		public:
			SysLogWriter(const std::string name);
			virtual ~SysLogWriter();
			
			virtual void show(LogLevel level = LogLevel::TRACE);	
			virtual void begin(std::ostringstream& os, LogLevel level, unsigned category);
			virtual void end(std::ostringstream& os);
			virtual void endl(std::ostringstream& os);
						
		private:
			std::string name;
			LogLevel level;
			LogLevel visibleLevel;
			bool enabled;
		};
	}
}

#endif /* ORG_EEROS_LOGGER_SYSLOGWRITER_HPP_ */
