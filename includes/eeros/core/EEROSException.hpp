#ifndef ORG_EEROS_CORE_EEROSEXCEPTION_HPP
#define ORG_EEROS_CORE_EEROSEXCEPTION_HPP

#include <string>
#include <exception>

namespace eeros {
	class EEROSException : public std::exception {

	public:
		EEROSException(std::string message);
		virtual ~EEROSException() throw();
		
		virtual const char* what() const throw();
		
	private:
		std::string message;
	};
};

#endif // ORG_EEROS_CORE_EEROSEXCEPTION_HPP
