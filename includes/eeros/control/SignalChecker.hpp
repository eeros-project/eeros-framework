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
			SignalChecker(T lowerLimit, T upperLimit) : 
				lowerLimit(lowerLimit), 
				upperLimit(upperLimit),
				fired(false) 
			{ }
			
			virtual void run() override {
				auto val = this->in.getSignal().getValue();
				if (!fired) {
					if (val < lowerLimit || val > upperLimit) {
						if(safetySystem != nullptr && safetyEvent != nullptr) {
							safetySystem->triggerEvent(*safetyEvent);
							fired = true;
						}
					}
				}
			}
			
			virtual void reset() {
				fired = false;
			}
			
			virtual void registerSafetyEvent(SafetySystem& ss, SafetyEvent& e) {
				safetySystem = &ss;
				safetyEvent = &e;
			}

		protected:
			T lowerLimit, upperLimit;
			bool fired;
			SafetySystem* safetySystem;
			SafetyEvent* safetyEvent;
		};
		
	};
};

#endif /* ORG_EEROS_SIGNAL_CHECKER_HPP_ */
