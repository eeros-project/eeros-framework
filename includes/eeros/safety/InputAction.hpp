#ifndef ORG_EEROS_SAFETY_INPUTACTION_HPP_
#define ORG_EEROS_SAFETY_INPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/Input.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetyContext.hpp>

namespace eeros {
	namespace safety {

		class SafetyContext;
		
		class InputAction {
		public:
			InputAction(hal::InputInterface& input) : input(&input) { }
			virtual ~InputAction() { }
			virtual bool check(SafetyContext* context) { return false; }
			virtual hal::InputInterface* getInput() {return input;}
		protected:
			hal::InputInterface* input;
		};

		template <typename T>
		class IgnoreInputAction : public InputAction {
		public:
			IgnoreInputAction(hal::Input<T>& input) : InputAction(input) { }
			virtual ~IgnoreInputAction() { }
			virtual bool check(SafetyContext* context) { return false; }
		};

		template <typename T>
		class CheckInputAction : public InputAction {
		public:
			CheckInputAction(hal::Input<T>& input, T value, SafetyEvent& event) : InputAction(input), value(value), event(event) { }
			virtual ~CheckInputAction() { }
			virtual bool check(SafetyContext* context) {
				if (dynamic_cast<hal::Input<T>*>(input)->get() != value) {
					context->triggerEvent(event);
					return true;
				}
				return false;
			}
		private:
			T value;
			SafetyEvent& event;
		};

		template <typename T>
		class CheckRangeInputAction : public InputAction {
		public:
			CheckRangeInputAction(hal::Input<T>& input, T min, T max, SafetyEvent& event) : InputAction(input), min(min), max(max), event(event) { }
			virtual ~CheckRangeInputAction() { }
			virtual bool check(SafetyContext* context) {
				T value = dynamic_cast<hal::Input<T>*>(input)->get();
				if (value < min || value > max) {
					context->triggerEvent(event);
					return true;
				}
				return false;
			}
		private:
			T min;
			T max;
			SafetyEvent& event;
		};

		template <typename T>
		IgnoreInputAction<T>* ignore(eeros::hal::Input<T>& input) {
			return new IgnoreInputAction<T>(input);
		}
		
		template <typename T>
		IgnoreInputAction<T>* ignore(eeros::hal::Input<T>* input) {
			return new IgnoreInputAction<T>(*input);
		}

		template <typename T>
		CheckInputAction<T>* check(eeros::hal::Input<T>& input, T value, SafetyEvent& event) {
			return new CheckInputAction<T>(input, value, event);
		}
		
		template <typename T>
		CheckInputAction<T>* check(eeros::hal::Input<T>* input, T value, SafetyEvent& event) {
			return new CheckInputAction<T>(*input, value, event);
		}

		template <typename T>
		CheckRangeInputAction<T>* range(eeros::hal::Input<T>& input, T min, T max, SafetyEvent& event) {
			return new CheckRangeInputAction<T>(input, min, max, event);
		}
		
		template <typename T>
		CheckRangeInputAction<T>* range(eeros::hal::Input<T>* input, T min, T max, SafetyEvent& event) {
			return new CheckRangeInputAction<T>(*input, min, max, event);
		}

	};
};

#endif // ORG_EEROS_SAFETY_INPUTACTION_HPP_
