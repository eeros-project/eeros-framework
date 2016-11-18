#ifndef ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
#define ORG_EEROS_SAFETY_OUTPUTACTION_HPP_

#include <stdint.h>
#include <memory>
#include <bits/algorithmfwd.h>
#include <eeros/hal/HAL.hpp>

namespace eeros {
	namespace safety {

		class OutputAction {
		public:
			OutputAction(hal::PeripheralOutputInterface* out) : output(out) { }
			virtual ~OutputAction() { }
			virtual void set() = 0;
			virtual hal::PeripheralOutputInterface* getOutput() {return output;}
		protected:
			hal::PeripheralOutputInterface* output;
		};
		
		template <typename T>
		class LeaveOutputAction : public OutputAction {
		public:
			LeaveOutputAction(hal::PeripheralOutput<T>* output) : OutputAction(output) { }
			virtual ~LeaveOutputAction() { }
			virtual void set() { }
		};

		template < typename T >
		class SetOutputAction : public OutputAction {
		public:
			SetOutputAction(hal::PeripheralOutput<T>* output, T value) : OutputAction(output), value(value) { }
			virtual ~SetOutputAction() { }
			virtual void set() { 
				dynamic_cast<hal::PeripheralOutput<T>*>(output)->set(value);
			}
		private:
			T value;
		};
	
		template < typename T >
		class ToggleOutputAction : public OutputAction {
		public:
			ToggleOutputAction(hal::PeripheralOutput<T>* output, T low, T high) : OutputAction(output), value(low), low(low), high(high) { }
			virtual ~ToggleOutputAction() { }
			virtual void set() {
				dynamic_cast<hal::PeripheralOutput<T>*>(output)->set(value);
				if (value == low)
					value = high;
				else
					value = low;
			}
		private:
			T value;
			T low;
			T high;
		};

		template <typename T>
		SetOutputAction<T>* set(eeros::hal::PeripheralOutput<T>& output, T value) {
			return new SetOutputAction<T>(output, value);
		}
		
		template <typename T>
		SetOutputAction<T>* set(eeros::hal::PeripheralOutput<T>* output, T value) {
			return new SetOutputAction<T>(output, value);
		}

		template <typename T>
		LeaveOutputAction<T>* leave(eeros::hal::PeripheralOutput<T>& output) {
			return new LeaveOutputAction<T>(output);
		}
		
		template <typename T>
		LeaveOutputAction<T>* leave(eeros::hal::PeripheralOutput<T>* output) {
			return new LeaveOutputAction<T>(output);
		}
		
		template <typename T>
		ToggleOutputAction<T>* toggle(eeros::hal::PeripheralOutput<T>& output, T low = false, T high = true) {
			return new ToggleOutputAction<T>(output, low, high );
		}
		
		template <typename T>
		ToggleOutputAction<T>* toggle(eeros::hal::PeripheralOutput<T>* output, T low = false, T high = true) {
			return new ToggleOutputAction<T>(output, low, high );
		}
	};
};

#endif // ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
