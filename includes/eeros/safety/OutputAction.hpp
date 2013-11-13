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
			LeaveOutputAction(eeros::hal::SystemOutput<T>& output) { }
			virtual ~LeaveOutputAction() { }
			virtual void set() { }
		private:
			eeros::hal::SystemOutput<T>& output;
		};

		template < typename T >
		class SetOutputAction : public OutputAction {
		public:
			SetOutputAction(eeros::hal::SystemOutput<T>& output, T value) : output(output), value(value) { }
			virtual ~SetOutputAction() { }
			virtual void set() { 
				output.set(value);
			}
		private:
			eeros::hal::SystemOutput<T>& output;
			T value;
		};

		template <typename T>
		SetOutputAction<T>* set(eeros::hal::SystemOutput<T>& output, T value) {
			return new SetOutputAction<T>(output, value);
		}
		
		template <typename T>
		SetOutputAction<T>* set(eeros::hal::SystemOutput<T>* output, T value) {
			return new SetOutputAction<T>(*output, value);
		}

		template <typename T>
		LeaveOutputAction<T>* leave(eeros::hal::SystemOutput<T>& output) {
			return new LeaveOutputAction<T>(output);
		}
		
		template <typename T>
		LeaveOutputAction<T>* leave(eeros::hal::SystemOutput<T>* output) {
			return new LeaveOutputAction<T>(*output);
		}

	};
};

#endif // ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
