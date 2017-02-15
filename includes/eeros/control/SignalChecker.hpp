#ifndef ORG_EEROS_SIGNAL_CHECKER_HPP_
#define ORG_EEROS_SIGNAL_CHECKER_HPP_

#include <eeros/control/Block1i.hpp>
#include <eeros/safety/SafetyLevel.hpp>

namespace eeros {
	namespace control {

		using namespace safety;
		
		template <typename T = double>
		class SignalChecker : public Block1i<T> {
		public:
			SignalChecker(T lowerLimit, T upperLimit, SafetySystem& ss, SafetyEvent& event) : 
				lowerLimit(lowerLimit), 
				upperLimit(upperLimit),
				ss(ss), 
				e(event), 
				fired(false) 
			{ }
			
			virtual void run() override {
				auto val = this->in.getSignal().getValue();
				if (!fired) {
					if (val < lowerLimit || val > upperLimit) {
						ss.triggerEvent(e);
						fired = true;
					}
				}
			}
			
			virtual void reset() {
				fired = false;
			}
			
		protected:
			T lowerLimit, upperLimit;
			bool fired;
			SafetySystem& ss;
			SafetyEvent& e;
		};
		
	};
};

#endif /* ORG_EEROS_SIGNAL_CHECKER_HPP_ */
