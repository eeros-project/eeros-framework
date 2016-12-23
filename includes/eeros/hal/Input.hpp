#ifndef ORG_EEROS_HAL_INPUT_HPP_
#define ORG_EEROS_HAL_INPUT_HPP_
#include <string>

namespace eeros {
	namespace hal {

		class InputInterface {
		public:
			virtual ~InputInterface() { }
			virtual std::string getId() const = 0;
			virtual void* getLibHandle() = 0;
		};

		template <typename T>
		class Input : public InputInterface {
		public:
			explicit Input(std::string id, void* libHandle) : id(id), libHandle(libHandle) { }
			virtual ~Input() { }
			virtual inline std::string getId() const { return id; }
			virtual T get() = 0;
			virtual void *getLibHandle() { return libHandle; }
		private:
			std::string id;
			void* libHandle;
		};

	};
};

#endif /* ORG_EEROS_HAL_INPUT_HPP_ */
