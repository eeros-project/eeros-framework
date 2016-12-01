#ifndef ORG_EEROS_HAL_OUTPUT_HPP_
#define ORG_EEROS_HAL_OUTPUT_HPP_

#include <eeros/core/System.hpp>
#include <string>

namespace eeros {
	namespace hal {

		class OutputInterface {
		public:
			virtual ~OutputInterface() { }
			virtual std::string getId() const = 0;
		};

		template <typename T>
		class Output : public OutputInterface {
		public:
			explicit Output(std::string id) : id(id) { }
			virtual ~Output() { }
			virtual inline std::string getId() const { return id; }
			virtual T get() = 0;
			virtual void set(T value) = 0;
		private:
			std::string id;
		};

	};
};

#endif /* ORG_EEROS_HAL_OUTPUT_HPP_ */
