#ifndef ORG_EEROS_SAFETY_EVENT_HPP_
#define ORG_EEROS_SAFETY_EVENT_HPP_

#include <string>

class Event {

public:
	Event(std::string name, bool priv = false);
	virtual ~Event();

	void makePrivate();
	void makePublic();
	
private:
	std::string name;
	bool priv;
};

#endif // ORG_EEROS_SAFETY_EVENT_HPP_