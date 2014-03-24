#ifndef ORG_EEROS_CONTROL_GAIN_HPP_
#define ORG_EEROS_CONTROL_GAIN_HPP_

#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {

		template < typename Tout = double, typename Tgain = double >
		class Gain : public Block1i1o<Tout> {
			
		public:
			Gain(Tgain c) : enabled(true) {
				gain = c;
			}
			
			virtual void run() {
				if(enabled) {
					this->out.getSignal().setValue(gain * this->in.getSignal().getValue());
				}
				else {
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

	};
};

#endif /* ORG_EEROS_CONTROL_GAIN_HPP_ */
