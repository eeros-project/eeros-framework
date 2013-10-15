#ifndef ORG_EEROS_HAL_SYSTEMINPUT_HPP_
#define ORG_EEROS_HAL_SYSTEMINPUT_HPP_

class SystemInputInterface { };

template <typename T>
class SystemInput : public SystemInputInterface {
public:
	explicit SystemInput(T& value) : value(value) { }
	inline T& get(void) { return value; }
private:
	T& value;
};

#endif /* ORG_EEROS_HAL_SYSTEMINPUT_HPP_ */