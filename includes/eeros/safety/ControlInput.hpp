#ifndef ORG_EEROS_SAFETY_CONTROLINPUT_HPP_
#define ORG_EEROS_SAFETY_CONTROLINPUT_HPP_


#include <stdint.h>

#include <eeros/control/Signal.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/hal/Input.hpp>


namespace eeros {
	namespace safety {

		template <typename T>
		class ControlInput : public hal::Input<T> {
		public:
			ControlInput(std::string id, control::Input<T> &input);
			ControlInput(std::string id, control::Output<T> &output);
			ControlInput(std::string id, control::Signal<T> &signal);
			virtual ~ControlInput();
			virtual T get();
		private:
			control::Signal<T> &signal;
		};
		
		template <typename T>
		ControlInput<T>::ControlInput(std::string id, control::Input<T> &input) :
			hal::Input<T>(id), signal(input.getSignal()) { }

		template <typename T>
		ControlInput<T>::ControlInput(std::string id, control::Output<T> &output) :
			hal::Input<T>(id), signal(output.getSignal()) { }

		template <typename T>
		ControlInput<T>::ControlInput(std::string id, control::Signal<T> &signal) :
			hal::Input<T>(id), signal(signal) { }
		
		template <typename T>
		ControlInput<T>::~ControlInput() { }
		
		template <typename T>
		T ControlInput<T>::get() { return signal.getValue(); }
		
	}
}

#endif // ORG_EEROS_SAFETY_CONTROLINPUT_HPP_
