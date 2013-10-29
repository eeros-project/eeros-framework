#ifndef ORG_EEROS_HAL_SYSTEMINPUT_HPP_
#define ORG_EEROS_HAL_SYSTEMINPUT_HPP_
#include <string>

class SystemInputInterface { public: virtual ~SystemInputInterface() {} };

template <typename T>
class SystemInput : public SystemInputInterface {
public:
	explicit SystemInput(std::string id) : id(id) { }
	virtual ~SystemInput() { }
	virtual T get() = 0;
private:
	std::string id;
};

#endif /* ORG_EEROS_HAL_SYSTEMINPUT_HPP_ */