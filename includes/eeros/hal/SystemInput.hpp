#ifndef ORG_EEROS_HAL_SYSTEMINPUT_HPP_
#define ORG_EEROS_HAL_SYSTEMINPUT_HPP_
#include <string>

namespace eeros {
	namespace hal {

		// TODO rename to PeripheralInput
		class SystemInputInterface {
		public:
			virtual ~SystemInputInterface() { }
			virtual std::string getId() = 0;
		};

		template <typename T>
		class SystemInput : public SystemInputInterface {
		public:
			explicit SystemInput(std::string id) : id(id) { }
			virtual ~SystemInput() { }
			virtual inline std::string getId() { return id; }
			virtual T get() = 0;
		private:
			std::string id;
		};

	};
};

#endif /* ORG_EEROS_HAL_SYSTEMINPUT_HPP_ */