#ifndef ORG_EEROS_LOGGER_LOGENTRY_HPP_
#define ORG_EEROS_LOGGER_LOGENTRY_HPP_
#include <eeros/logger/LogWriter.hpp>

namespace eeros {
	namespace logger {
		
		class LogEntry {
		public:
			LogEntry(LogWriter* writer, LogLevel level, unsigned category = 0) : w(writer) {
				if(w != nullptr) w->begin(os, level, category);
			}
			LogEntry(const LogEntry&);
			
			virtual ~LogEntry() {
				if(w != nullptr) w->end(os);
			}

			template <typename T>
			LogEntry& operator<<(T value) {
				os << value;
				return *this;
			}
			
			LogEntry& operator<<(void (*f)(LogWriter&) ) {
				w->endl(os);
				return *this;
			}
		private:
			LogWriter* w;
			std::ostringstream os;
		};
		
	}
}

#endif /* ORG_EEROS_LOGGER_LOGENTRY_HPP_ */
