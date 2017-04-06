#ifndef ORG_EEROS_CONTROL_SWITCH_HPP_
#define ORG_EEROS_CONTROL_SWITCH_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetySystem.hpp>
// #include <cmath>

namespace eeros {
	namespace control {
		using namespace safety;
		
		template < uint8_t N = 2, typename T = double >
                class Switch : public Block1o<T> {
                public:
                        Switch(uint8_t initInputIndex) : currentInput(initInputIndex) {
                        }
                       
                        virtual void run() {
				auto val = this->in[currentInput].getSignal().getValue();
				if (armed && !switched) {
					if (val < (switchLevel + delta) && val > (switchLevel - delta)) {
						switchToInput(nextInput);
						switched = true;
						armed = false;
						if(safetySystem != nullptr && safetyEvent != nullptr) {
							safetySystem->triggerEvent(*safetyEvent);
						}
					}
				}

                                this->out.getSignal().setValue(this->in[currentInput].getSignal().getValue());
                                this->out.getSignal().setTimestamp(this->in[currentInput].getSignal().getTimestamp());
                        }
                       
                        virtual Input<T>& getIn(uint8_t index) {
                                return in[index];
                        }
                                               
                        virtual bool switchToInput(uint8_t index) {
                                if(index >= 0 && index < N) {
                                        currentInput = index;
                                        return true;
                                }
                                return false;
                        }
                       
                        virtual uint8_t getCurrentInput() const {
                                return currentInput;
                        }
                       
			virtual void registerSafetyEvent(SafetySystem* ss, SafetyEvent* e) {
				safetySystem = ss;
				safetyEvent = e;
			}

                        virtual void setCondition(T switchLevel, T delta, uint8_t index) {
				this->switchLevel = switchLevel;
                                this->delta = delta;
				nextInput = index;
                        }
                       
                        virtual void arm() {
                                armed = true;
				switched = false;
                        }
                       
                        virtual bool triggered() const {
                                return switched;
                        }
                       
                protected:
                        Input<T> in[N];
                        uint8_t currentInput, nextInput;
			T switchLevel, delta;
                        bool armed = false;
                        bool switched = false;
			SafetySystem* safetySystem = nullptr;
			SafetyEvent* safetyEvent = nullptr;
                };
	};
};

#endif /* ORG_EEROS_CONTROL_SWITCH_HPP_ */
