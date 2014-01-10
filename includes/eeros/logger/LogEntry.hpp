#ifndef ORG_EEROS_LOGGER_LOGENTRY_HPP_
#define ORG_EEROS_LOGGER_LOGENTRY_HPP_

namespace eeros
{
	namespace logger
	{
		template < typename TWriter >
		class LogEntry
		{
		public:
			LogEntry(TWriter* writer, unsigned level, unsigned category = 0) :
				w(writer)
			{
				if (w != nullptr)
					w->begin(level, category);
			}

			~LogEntry()
			{
				if (w != nullptr)
					w->end();
			}

			template < typename T >
			LogEntry<TWriter>& operator <<(T value)
			{
				if (w != nullptr)
					(*w) << value;

				return *this;
			}

			template < typename ... T >
			void write(T ... arg)
			{
				if (w != nullptr)
					w->write(arg...);
			}

			void put() { }

			template < typename T, typename ... TArg >
			void put(T arg, TArg ... args)
			{
				(*w) << arg;
				if (w != nullptr)
				{
					put(args...);
				}
			}


			TWriter* w;
		};
	}
}

#endif /* ORG_EEROS_LOGGER_LOGENTRY_HPP_ */
