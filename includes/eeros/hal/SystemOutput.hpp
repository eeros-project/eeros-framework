#ifndef ORG_EEROS_HAL_SYSTEMOUTPUT_HPP_
#define ORG_EEROS_HAL_SYSTEMOUTPUT_HPP_

#include <eeros/hal/SystemInput.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
	namespace hal {

		// TODO rename to PeripheralOutput
		class SystemOutputInterface {
		public:
			virtual ~SystemOutputInterface() { }
			virtual std::string getId() = 0;
		};

		template <typename T>
		class SystemOutput : public SystemOutputInterface {
		public:
			explicit SystemOutput(std::string id) : id(id) { }
			virtual ~SystemOutput() { }
			virtual inline std::string getId() { return id; }
			virtual T get() = 0;
			virtual void set(T value) = 0;
		//	inline SystemInput<T> getInput(void) { return SystemInput<T>(value); }
		private:
			std::string id;
		};

	};
};

#endif /* ORG_EEROS_HAL_SYSTEMOUTPUT_HPP_ */