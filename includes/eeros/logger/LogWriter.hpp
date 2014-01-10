#ifndef ORG_EEROS_LOGGER_LOGWRITER_HPP_
#define ORG_EEROS_LOGGER_LOGWRITER_HPP_

#include <eeros/logger/Writer.hpp>
#include <string>

namespace eeros
{
	namespace logger
	{
		class LogWriter : public Writer
		{
		public:
			virtual ~LogWriter() { }
			virtual void endl() = 0;
			virtual LogWriter& operator <<(int value) = 0;
			virtual LogWriter& operator <<(double value) = 0;
			virtual LogWriter& operator <<(const std::string& value) = 0;
			virtual LogWriter& operator <<(void (*f)(LogWriter&)) = 0;
		};

		void endl(LogWriter& w);
	}
}

#endif /* ORG_EEROS_LOGGER_LOGWRITER_HPP_ */
