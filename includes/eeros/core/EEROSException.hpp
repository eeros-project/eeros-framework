#ifndef ORG_EEROS_CORE_EEROSEXCEPTION_HPP
#define ORG_EEROS_CORE_EEROSEXCEPTION_HPP

#include <string>
#include <sstream>
#include <exception>

namespace eeros {
	class EEROSException : public std::exception {

	public:
		EEROSException();
		explicit EEROSException(std::string m);
		virtual ~EEROSException() throw();
		
		virtual const char* what() const throw();
		
	protected:
		std::string message;
	};
};

#endif // ORG_EEROS_CORE_EEROSEXCEPTION_HPP
