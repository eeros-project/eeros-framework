#ifndef ORG_EEROS_SAFETY_CONTROLINPUT_HPP_
#define ORG_EEROS_SAFETY_CONTROLINPUT_HPP_


#include <stdint.h>

#include <eeros/control/Signal.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/hal/PeripheralInput.hpp>


namespace eeros {
	namespace safety {

		template <typename T>
		class ControlInput : public hal::PeripheralInput<T> {
		public:
			explicit ControlInput(std::string id, control::Input<T> &input);
			virtual ~ControlInput();
			virtual T get();
		private:
			control::Signal<T> &input;
		};
		
		template <typename T>
		ControlInput<T>::ControlInput(std::string id, control::Input<T> &input) : hal::PeripheralInput<T>(id), input(input.getSignal()) { }
		
		template <typename T>
		ControlInput<T>::~ControlInput() { }
		
		template <typename T>
		T ControlInput<T>::get() { return input.getValue(); }
		
	}
}

#endif // ORG_EEROS_SAFETY_CONTROLINPUT_HPP_
