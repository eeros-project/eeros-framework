#ifndef ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_
#define ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_

#include <eeros/logger/LogWriter.hpp>
#include <ostream>
#include <mutex>

namespace eeros
{
	namespace logger
	{
		class StreamLogWriter : public LogWriter
		{
		public:
			StreamLogWriter(std::ostream& out);
			
			virtual void show(unsigned level = ~0);
			
			virtual void begin(unsigned level, unsigned category);
			virtual void end();
			
			virtual void endl();
			
			virtual LogWriter& operator<<(int value);
			virtual LogWriter& operator<<(unsigned int value);
			virtual LogWriter& operator<<(long value);
			virtual LogWriter& operator<<(double value);
			virtual LogWriter& operator<<(const std::string& value);
			virtual LogWriter& operator<<(std::ostream& os);
			virtual LogWriter& operator<<(void (*f)(LogWriter&));
			
		private:
			std::mutex mtx;
			std::ostream& out;
			unsigned visible_level;
			bool enabled;
			bool colored;
		};
	}
}

#endif /* ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_ */
