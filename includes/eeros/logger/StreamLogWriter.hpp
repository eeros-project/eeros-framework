#ifndef ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_
#define ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_

#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/Logger.hpp>

namespace eeros {
	namespace logger {
		class StreamLogWriter : public LogWriter {
		public:
			StreamLogWriter(std::ostream& out);
			
			virtual void show(LogLevel level = LogLevel::TRACE);	
			virtual void begin(std::ostringstream& os, LogLevel level, unsigned category);
			virtual void end(std::ostringstream& os);
			virtual void endl(std::ostringstream& os);
						
		private:
			std::ostream& out;
			LogLevel visible_level;
			bool enabled;
			bool colored;
		};
				
	}
}

#endif /* ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_ */
