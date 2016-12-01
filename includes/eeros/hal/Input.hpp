#ifndef ORG_EEROS_HAL_INPUT_HPP_
#define ORG_EEROS_HAL_INPUT_HPP_
#include <string>

namespace eeros {
	namespace hal {

		class InputInterface {
		public:
			virtual ~InputInterface() { }
			virtual std::string getId() const = 0;
		};

		template <typename T>
		class Input : public InputInterface {
		public:
			explicit Input(std::string id) : id(id) { }
			virtual ~Input() { }
			virtual inline std::string getId() const { return id; }
			virtual T get() = 0;
		private:
			std::string id;
		};

	};
};

#endif /* ORG_EEROS_HAL_INPUT_HPP_ */
