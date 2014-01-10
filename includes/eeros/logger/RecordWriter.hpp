#ifndef ORG_EEROS_LOGGER_RECORDWRITER_HPP_
#define ORG_EEROS_LOGGER_RECORDWRITER_HPP_

#include <eeros/logger/Writer.hpp>

namespace eeros
{
	namespace logger
	{
		template < typename TRecord >
		class RecordWriter : public Writer
		{
		public:
			virtual ~RecordWriter() { }
			virtual void endl() = 0;
			virtual LogWriter& operator <<(TRecord r) = 0;
		};

		void endl(LogWriter& w) { w.endl(); }
	}
}

#endif /* ORG_EEROS_LOGGER_RECORDWRITER_HPP_ */
