#ifndef ORG_EEROS_LOGGER_LOGGER_HPP_
#define ORG_EEROS_LOGGER_LOGGER_HPP_

#include <eeros/logger/LogEntry.hpp>

namespace eeros {
	namespace logger {
		
		template < typename TWriter >
		class Logger {
		public:
			Logger(unsigned category = 0) : w(defaultWriter), category(category) { }
			
			void set(TWriter* writer) { w = writer; }
			void set(TWriter& writer) { w = &writer; }
			TWriter* get() { return w; }
			
			LogEntry<TWriter> fatal() { return LogEntry<TWriter>(w, 0, category); }
			LogEntry<TWriter> error() { return LogEntry<TWriter>(w, 1, category); }
			LogEntry<TWriter> warn()  { return LogEntry<TWriter>(w, 2, category); }
			LogEntry<TWriter> info()  { return LogEntry<TWriter>(w, 3, category); }
			LogEntry<TWriter> trace() { return LogEntry<TWriter>(w, 4, category); }
			
			LogEntry<TWriter> operator <<(unsigned level) { return LogEntry<TWriter>(w, level, category); }
			
			
			static void setDefaultWriter(TWriter* writer) { defaultWriter = writer; }
		private:
			TWriter* w;
			unsigned category;
			
			static TWriter* defaultWriter;
		};
		
		template < typename TWriter >
		TWriter* Logger<TWriter>::defaultWriter = nullptr;
	}
}

#endif /* ORG_EEROS_LOGGER_LOGGER_HPP_ */
