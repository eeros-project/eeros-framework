#ifndef ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_
#define ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_

#include <eeros/core/Lock.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <ostream>

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

			virtual LogWriter& operator <<(int value);
			virtual LogWriter& operator <<(double value);
			virtual LogWriter& operator <<(const std::string& value);
			virtual LogWriter& operator <<(void (*f)(LogWriter&));

		private:
			Mutex mutex;
			Lock lock;
			std::ostream& out;
			unsigned visible_level;
			bool enabled;
			bool colored;
		};
	}
}

#endif /* ORG_EEROS_LOGGER_STREAMLOGWRITER_HPP_ */
