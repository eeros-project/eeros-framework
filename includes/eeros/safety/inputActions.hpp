#ifndef ORG_EEROS_SAFETY_INPUTACTIONS_HPP_
#define ORG_EEROS_SAFETY_INPUTACTIONS_HPP_

#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/SafetyContext.hpp>

namespace eeros {
	namespace safety {

		template <typename T>
		class IgnoreInputAction : public InputAction {
		public:
			IgnoreInputAction(eeros::hal::PeripheralInput<T>& input) : InputAction(input), input(input) { }
			virtual ~IgnoreInputAction() { }
			virtual bool check(SafetyContext* context) { return false; }

		private:
			eeros::hal::PeripheralInput<T>& input;
		};

		template <typename T>
		class CheckInputAction : public InputAction {
		public:
			CheckInputAction(eeros::hal::PeripheralInput<T>& input, T value, SafetyEvent event) : InputAction(input), input(input), value(value), event(event) { }
			virtual ~CheckInputAction() { }
			virtual bool check(SafetyContext* context) {
				if(input.get() != value) {
					context->triggerEvent(event);
					return true;
				}
				return false;
			}
			
		private:
			eeros::hal::PeripheralInput<T>& input;
			T value;
			SafetyEvent event;
		};

		template <typename T>
		class CheckRangeInputAction : public InputAction {
		public:
			CheckRangeInputAction(eeros::hal::PeripheralInput<T>& input, T min, T max, SafetyEvent event) : InputAction(input), input(input), min(min), max(max), event(event) { }
			virtual ~CheckRangeInputAction() { }
			virtual bool check(SafetyContext* context) {
				T value = input.get();
				if (value < min || value > max) {
					context->triggerEvent(event);
					return true;
				}
				return false;
			}

		private:
			eeros::hal::PeripheralInput<T>& input;
			T min;
			T max;
			SafetyEvent event;
		};

		template <typename T>
		IgnoreInputAction<T>* ignore(eeros::hal::PeripheralInput<T>& input) {
			return new IgnoreInputAction<T>(input);
		}
		
		template <typename T>
		IgnoreInputAction<T>* ignore(eeros::hal::PeripheralInput<T>* input) {
			return new IgnoreInputAction<T>(*input);
		}

		template <typename T>
		CheckInputAction<T>* check(eeros::hal::PeripheralInput<T>& input, T value, SafetyEvent event) {
			return new CheckInputAction<T>(input, value, event);
		}
		
		template <typename T>
		CheckInputAction<T>* check(eeros::hal::PeripheralInput<T>* input, T value, SafetyEvent event) {
			return new CheckInputAction<T>(*input, value, event);
		}

		template <typename T>
		CheckRangeInputAction<T>* range(eeros::hal::PeripheralInput<T>& input, T min, T max, SafetyEvent event) {
			return new CheckRangeInputAction<T>(input, min, max, event);
		}
		
		template <typename T>
		CheckRangeInputAction<T>* range(eeros::hal::PeripheralInput<T>* input, T min, T max, SafetyEvent event) {
			return new CheckRangeInputAction<T>(*input, min, max, event);
		}

	};
};

#endif // ORG_EEROS_SAFETY_INPUTACTIONS_HPP_
