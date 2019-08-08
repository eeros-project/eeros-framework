#ifndef ORG_EEROS_CONTROL_SWITCH_HPP_
#define ORG_EEROS_CONTROL_SWITCH_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <mutex>

namespace eeros {
	namespace control {
		using namespace safety;

		/**
		 * A switch allows to select a signal from severel inputs to be routed to the output.
		 * If a signal at a chosen input is close to a predefined value, the switch can automatically switch 
		 * to a predefined position and a safety event can be triggered.
		 * Two or more switches can be connected together. This mechanism allows to switch them simultaneously.
		 * 
		 * @tparam N - number of inputs
		 * @tparam T - value type (double - default type)
		 * 
		 * @since v0.4
		 */
		
		template < uint8_t N = 2, typename T = double >
		class Switch : public Block1o<T> {
		public:

			/**
			 * Constructs a Switch instance with the switch initialized to position 0.\n
			 */
			Switch() : currentInput(0) {
				for(uint8_t i = 0; i < N; i++) in[i].setOwner(this);
			}
                       
			/**
			 * Constructs a Switch instance with the switch initialized to a given position.\n
			 * @param initInputIndex - switch is initially set to this position
			 */
			Switch(uint8_t initInputIndex) : currentInput(initInputIndex) {
				for(uint8_t i = 0; i < N; i++) in[i].setOwner(this);
			}
                       
			/**
			 * Runs the switch.
			 */
			virtual void run() {
				auto val = this->in[currentInput].getSignal().getValue();
				if (armed && !switched) {
					if (val < (switchLevel + delta) && val > (switchLevel - delta)) {
						switchToInput(nextInput);
						for(Switch* i : c) i->switchToInput(nextInput);
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
                       
			/**
			* Getter function for the input with a given index.
			* 
			* @tparam index - index of input
			* @return The input with this index
			*/
			virtual Input<T>& getIn(uint8_t index) {
				return in[index];
			}
                                               
			/**
			* Changes the switch position.
			* 
			* @tparam index - position to switch to
			* @return true, if operation successful
			*/
			virtual bool switchToInput(uint8_t index) {
				if(index >= 0 && index < N) {
					currentInput = index;
					for(Switch* i : c) i->switchToInput(index);
					return true;
				}
				return false;
			}
                       
			/**
			* Gets the actual position of the switch.
			* 
			* @return current position
			*/
			virtual uint8_t getCurrentInput() const {
				return currentInput;
			}
                       
			/**
			* Registers a safety system together with a safety event, which will be
			* triggered as soon as auto switching takes place and the trigger was armed beforehand.
			 * @see setCondition()
			* 
			* @tparam ss - safety system
			* @tparam e - safety event
			*/
			virtual void registerSafetyEvent(SafetySystem& ss, SafetyEvent& e) {
				safetySystem = &ss;
				safetyEvent = &e;
			}

			/**
			* Configure the switch so that it switches only after the input signal is close 
			* to a given level within a certain margin. Switching will then happen automatically
			* if the switch is armed.
			* It is possible to trigger a safety event upon switching
			 * @see registerSafetyEvent()
			* 
			* @tparam switchLevel - level, that the input signal has to reach
			* @tparam delta - level margin, that the input signal has to reach
			* @tparam index - position to switch to 
			*/
			virtual void setCondition(T switchLevel, T delta, uint8_t index) {
				this->switchLevel = switchLevel;
				this->delta = delta;
				nextInput = index;
			}
                       
			/**
			* Allows to switch automatically.
			* @see setCondition()
			*/
			virtual void arm() {
				armed = true;
				switched = false;
			}

			/**
			* Reads the state of the trigger mechanism.
			* @see setCondition()
			* 
			* @return true, if safety event was fired
			*/
			virtual bool triggered() const {
				return switched;
			}
                       
			/**
			* Connects a second switch to this switch. The second switch will 
			* automatically be on the same position as the this switch.
			* triggered as soon as auto switching takes place.
			* 
			* @tparam s - switch to be connected to this switch
			*/
			virtual void connect(Switch& s) {
				c.push_back(&s);
				s.switchToInput(currentInput);
			}
			
		protected:
			Input<T> in[N];
			uint8_t currentInput, nextInput;
			T switchLevel, delta;
			bool armed = false;
			bool switched = false;
			SafetySystem* safetySystem = nullptr;
			SafetyEvent* safetyEvent = nullptr;
			std::vector<Switch*> c;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_SWITCH_HPP_ */
