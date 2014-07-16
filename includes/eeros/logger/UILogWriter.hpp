#ifndef ORG_EEROS_LOGGER_UILOGWRITER_HPP_
#define ORG_EEROS_LOGGER_UILOGWRITER_HPP_

#include <eeros/core/Lock.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/ui/BaseUI.hpp>
#include <sstream>

namespace eeros {
	namespace logger {
		class UILogWriter : public LogWriter {
		
		public:
			UILogWriter(eeros::ui::BaseUI* ui);
			virtual ~UILogWriter();

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
		private:
			std::ostringstream out;
			unsigned level;
			Mutex mutex;
			Lock lock;
			unsigned visibleLevel;
			bool enabled;
		};
	}
}

#endif /* ORG_EEROS_LOGGER_UILOGWRITER_HPP_ */
