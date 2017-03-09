#ifndef ORG_EEROS_CONTROL_STEP_HPP_
#define ORG_EEROS_CONTROL_STEP_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Step : public Block1o<T> {
			
		public:
			Step(T initValue = 0, T stepHeight = 1, double delayTime = 1) : initValue(initValue), stepHeight(stepHeight), delayTime(delayTime) {
				first = true;
			}
			
			
			virtual void run() {
				if(first) {
					delayTime += System::getTime();
					this->out.getSignal().setValue(initValue);
					first = false;
				}
				else if(!stepDone && System::getTime() >= delayTime) {
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
				this->delayTime = delayTime;
			}

			template <typename X>
			friend std::ostream& operator<<(std::ostream& os, Step<X>& step);
			
		protected:
			T initValue;
			T stepHeight;
			double delayTime;
			bool stepDone;
			bool first;
		};

		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, Step<T>& step) {
			os << "Block step: '" << step.getName() << "' init val = " << step.initValue << " step height = " << step.stepHeight << " delay time = " << step.delayTime; 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_STEP_HPP_ */
