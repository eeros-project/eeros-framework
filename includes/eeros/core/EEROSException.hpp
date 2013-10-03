#ifndef EEROSEXCEPTION_HPP
#define EEROSEXCEPTION_HPP

#include <string>
#include <exception>

class EEROSException : public std::exception
{

public:
    EEROSException(std::string message);
	virtual ~EEROSException() throw();
	
	virtual const char* what();
	
private:
	std::string message;
};

#endif // EEROSEXCEPTION_HPP
