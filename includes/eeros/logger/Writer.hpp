#ifndef ORG_EEROS_LOGGER_WRITER_HPP_
#define ORG_EEROS_LOGGER_WRITER_HPP_

#include <sstream>

namespace eeros {
	namespace logger {
		enum class LogLevel;
		
		class Writer {
		public:
			virtual ~Writer() { }
			virtual void begin(std::ostringstream& os, LogLevel level, unsigned category) = 0;
			virtual void end(std::ostringstream& os) = 0;
		};
	}
}

#endif /* ORG_EEROS_LOGGER_WRITER_HPP_ */
