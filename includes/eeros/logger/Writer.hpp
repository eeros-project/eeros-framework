#ifndef ORG_EEROS_LOGGER_WRITER_HPP_
#define ORG_EEROS_LOGGER_WRITER_HPP_

namespace eeros
{
	namespace logger
	{
		class Writer
		{
		public:
			virtual ~Writer() { }
			virtual void begin(unsigned level, unsigned category) = 0;
			virtual void end() = 0;
		};
	}
}

#endif /* ORG_EEROS_LOGGER_WRITER_HPP_ */
