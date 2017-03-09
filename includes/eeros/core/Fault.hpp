#ifndef ORG_EEROS_CORE_FAULT_HPP
#define ORG_EEROS_CORE_FAULT_HPP

#include <string>
#include <sstream>
#include <exception>

namespace eeros {
	class Fault : public std::exception {

	public:
		Fault();
		explicit Fault(std::string m);
		virtual ~Fault() throw();
		
		virtual const char* what() const throw();
		
	protected:
		std::string message;
	};
};

#endif // ORG_EEROS_CORE_FAULT_HPP
