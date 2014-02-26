#ifndef ORG_EEROS_CONTROL_STEP_HPP_
#define ORG_EEROS_CONTROL_STEP_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Step : public Block1o<T> {
			
		public:
			Step(T initValue = 0, T stepHeight = 1, double delayTime = 1) {
				
			}
			
			
			virtual void run() {
				if(first) {
					stepTime += System::getTime();
					this->out.getSignal().setValue(initValue);
					first = false;
				}
				else if(!stepDone && System::getTime() >= stepTime) {
					this->out.getSignal().setValue(initValue + stepHeight);
					stepDone = true;
				}
				this->out.getSignal().setTimestamp(System::getTimeNs());
			}
			
			virtual void reset() {
				stepDone = false;
				first = true;
			}
			
			virtual void setInitValue(T initValue) {
				this->initValue = initValue;
			}
			
			virtual void setStepHeight(T stepHeight) {
				this->stepHeight = stepHeight;
			}
			
			virtual void setDelayTime(double delayTime) {
				this->stepTime = delayTime;
			}

		protected:
			T initValue;
			T stepHeight;
			double stepTime;
			bool stepDone;
			bool first;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_STEP_HPP_ */
