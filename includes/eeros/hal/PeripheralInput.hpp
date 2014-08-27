#ifndef ORG_EEROS_HAL_PERIPHERALINPUT_HPP_
#define ORG_EEROS_HAL_PERIPHERALINPUT_HPP_
#include <string>

namespace eeros {
	namespace hal {

		class PeripheralInputInterface {
		public:
			virtual ~PeripheralInputInterface() { }
			virtual std::string getId() const = 0;
		};

		template <typename T>
		class PeripheralInput : public PeripheralInputInterface {
		public:
			explicit PeripheralInput(std::string id) : id(id) { }
			virtual ~PeripheralInput() { }
			virtual inline std::string getId() const { return id; }
			virtual T get() = 0;
		private:
			std::string id;
		};

	};
};

#endif /* ORG_EEROS_HAL_PERIPHERALINPUT_HPP_ */
