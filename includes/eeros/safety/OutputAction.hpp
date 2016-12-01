#ifndef ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
#define ORG_EEROS_SAFETY_OUTPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/HAL.hpp>

namespace eeros {
	namespace safety {

		class OutputAction {
		public:
			virtual ~OutputAction() { }
			virtual void set() { }
		};

		template < typename T >
		class LeaveOutputAction : public OutputAction {
		public:
			LeaveOutputAction(eeros::hal::Output<T>& output) : output(output) { }
			virtual ~LeaveOutputAction() { }
			virtual void set() { }
		private:
			eeros::hal::Output<T>& output;
		};

		template < typename T >
		class SetOutputAction : public OutputAction {
		public:
			SetOutputAction(eeros::hal::Output<T>& output, T value) : output(output), value(value) { }
			virtual ~SetOutputAction() { }
			virtual void set() { 
				output.set(value);
			}
		private:
			eeros::hal::Output<T>& output;
			T value;
		};
	
		template < typename T >
		class ToggleOutputAction : public OutputAction {
		public:
			ToggleOutputAction(eeros::hal::Output<T>& output, T low, T high) : output(output), value(low), low(low), high(high) { }
			virtual ~ToggleOutputAction() { }
			virtual void set() {
				output.set(value);
				if (value == low)
					value = high;
				else
					value = low;
			}
		private:
			eeros::hal::Output<bool>& output;
			T value;
			T low;
			T high;
		};

		template <typename T>
		SetOutputAction<T>* set(eeros::hal::Output<T>& output, T value) {
			return new SetOutputAction<T>(output, value);
		}
		
		template <typename T>
		SetOutputAction<T>* set(eeros::hal::Output<T>* output, T value) {
			return new SetOutputAction<T>(*output, value);
		}

		template <typename T>
		LeaveOutputAction<T>* leave(eeros::hal::Output<T>& output) {
			return new LeaveOutputAction<T>(output);
		}
		
		template <typename T>
		LeaveOutputAction<T>* leave(eeros::hal::Output<T>* output) {
			return new LeaveOutputAction<T>(*output);
		}
		
		template <typename T>
		ToggleOutputAction<T>* toggle(eeros::hal::Output<T>* output, T low = false, T high = true) {
			return new ToggleOutputAction<T>(*output, low, high );
		}
	};
};

#endif // ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
