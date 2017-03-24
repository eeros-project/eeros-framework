#ifndef ORG_EEROS_CONTROL_GAIN_HPP_
#define ORG_EEROS_CONTROL_GAIN_HPP_

#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {

		template <typename Tout = double, typename Tgain = double, bool elementWise = false>
		class Gain : public Block1i1o<Tout> {
			
		public:
			Gain() : enabled(true) {
				gain = 1;
			}
			
			Gain(Tgain c) : enabled(true) {
				gain = c; 
			}
			
			virtual void run() {
				if(enabled) {
					this->out.getSignal().setValue(gain * this->in.getSignal().getValue());
				} else {
					this->out.getSignal().setValue(this->in.getSignal().getValue());
				}
				this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
			}
			
			virtual void enable() {
				enabled = true;
			}
			
			virtual void disable() {
				enabled = false;
			}
			
			virtual void setGain(Tgain c) {
				gain = c;
			}

			template <typename Xout, typename Xgain>
			friend std::ostream& operator<<(std::ostream& os, Gain<Xout,Xgain>& gain);

		protected:
			Tgain gain;
			bool enabled;
		};
		
		template <typename Tout, typename Tgain>
		class Gain<Tout, Tgain, true> : public Block1i1o<Tout> {
		
		public:
			Gain() : enabled(true) {
				gain = 1;
			}
			
			Gain(Tgain c) : enabled(true) {
				gain = c;
			}
			
			virtual void run() {
				if(enabled) {
					this->out.getSignal().setValue(this->in.getSignal().getValue().multiplyElementWise(gain));
				} else {
					this->out.getSignal().setValue(this->in.getSignal().getValue());
				}
				this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
			}
			
			virtual void enable() {
				enabled = true;
			}
			
			virtual void disable() {
				enabled = false;
			}
			
			virtual void setGain(Tgain c) {
				gain = c;
			}

		protected:
			Tgain gain;
			bool enabled;
		};
		
				/********** Print functions **********/
		template <typename Tout, typename Tgain>
		std::ostream& operator<<(std::ostream& os, Gain<Tout,Tgain>& gain) {
			os << "Block gain: '" << gain.getName() << "' gain val = " << gain.gain; 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_GAIN_HPP_ */
