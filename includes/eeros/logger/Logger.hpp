#ifndef ORG_EEROS_LOGGER_LOGGER_HPP_
#define ORG_EEROS_LOGGER_LOGGER_HPP_

#include <eeros/logger/LogEntry.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <sstream>
#include <string>
#include <iostream>

namespace eeros {
	namespace logger {
		
		enum class LogLevel {FATAL, ERROR, WARN, INFO, TRACE};
		
		class Logger {
		public:
			Logger(unsigned category = 0) : w(defaultWriter), category(category) { }
			virtual ~Logger() { }
			
			void set(LogWriter* writer) { w = writer; }
			void set(LogWriter& writer) { w = &writer; }
			LogWriter* get() { return w; }
			
			LogEntry fatal() { return LogEntry(w, LogLevel::FATAL, category); }
			LogEntry error() { return LogEntry(w, LogLevel::ERROR, category); }
			LogEntry warn() { return LogEntry(w, LogLevel::WARN, category); }
			LogEntry info() { return LogEntry(w, LogLevel::INFO, category); }
			LogEntry trace() { return LogEntry(w, LogLevel::TRACE, category); }
			
			static void setDefaultWriter(LogWriter* writer) { defaultWriter = writer; }
		private:
			LogWriter* w;
			unsigned category;
			static LogWriter* defaultWriter;
		};
	}
}

#endif /* ORG_EEROS_LOGGER_LOGGER_HPP_ */
