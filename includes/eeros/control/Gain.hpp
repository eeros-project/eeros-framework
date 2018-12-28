
#ifndef ORG_EEROS_CONTROL_GAIN_HPP_
#define ORG_EEROS_CONTROL_GAIN_HPP_

#include <eeros/control/Block1i1o.hpp>
namespace eeros {
	namespace control {

		template <typename Tout = double, typename Tgain = double, bool elementWise = false>
		class Gain : public Block1i1o<Tout> {
			
		public:
			Gain() {
				gain = 1;
				maxGain = 1000;
				minGain = -1000;
				targetGain = gain;
				gainDiff = 0;
			}

			Gain(Tgain c) {
				gain = c; 
				maxGain = 1000;
				minGain = -1000;
				targetGain = gain;
				gainDiff = 0;
			}

			Gain(Tgain c, Tgain maxGain, Tgain minGain = 1) {
				gain = c;
				this->maxGain = maxGain;
				this->minGain = minGain;
				targetGain = gain;
				gainDiff = 0;
			}

			virtual void run() {
				if(smoothChange){
				    if(gain < targetGain){
					gain += gainDiff;
					if(gain > targetGain){ //overshoot case
					  gain = targetGain;
					}
				    }
    
				    if(gain > targetGain){
					gain -= gainDiff;
					if(gain < targetGain){
					  gain = targetGain;
					}
				    }
				}  
    
				if(gain > maxGain){ //if diff will cause gain to be too large.
				    gain = maxGain;
				}

				if(gain < minGain){
				    gain = minGain;
				}
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

			virtual void enableSmoothChange(bool enable){
				smoothChange = enable;
			}

			virtual void setGain(Tgain c) {
				if(c <= maxGain && c >= minGain) {
				  if(smoothChange) {
				    targetGain = c;
				  }else {
				    gain = c;
				  }
				}
			}

			virtual void setMaxGain(Tgain maxGain) {
				this->maxGain = maxGain;
			}
			
			virtual void setMinGain(Tgain minGain) {
				this->minGain = minGain;
			}

			virtual void setGainDiff(Tgain gainDiff) {
				    this->gainDiff = gainDiff;
			}
			
			template <typename Xout, typename Xgain>
			friend std::ostream& operator<<(std::ostream& os, Gain<Xout,Xgain>& gain);

		protected:
			Tgain gain;
			Tgain maxGain;
			Tgain minGain;
			Tgain targetGain;
			Tgain gainDiff;
			bool enabled{true};
			bool smoothChange{false};
		};
		
		template <typename Tout, typename Tgain>
		class Gain<Tout, Tgain, true> : public Block1i1o<Tout> {
		
		public:
			Gain() {
				gain = 1;
				maxGain = 1000;
				minGain = -1000;
				targetGain = gain;
				gainDiff = 0;
			}
			
			Gain(Tgain c) {
				gain = c; 
				maxGain = 1000;
				minGain = -1000;
				targetGain = gain;
				gainDiff = 0;
			}
			
			Gain(Tgain c, Tgain maxGain, Tgain minGain = 1) {
				gain = c;
				this->maxGain = maxGain;
				this->minGain = minGain;
				targetGain = gain;
				gainDiff = 0;
			}
			
			virtual void run() {
				if(smoothChange){
				    if(gain < targetGain){
					gain += gainDiff;
					if(gain > targetGain){ //overshoot case
					  gain = targetGain;
					}
				    }
				    
				    if(gain > targetGain){
				      gain -= gainDiff;
				      if(gain < targetGain){
					  gain = targetGain;
					}
				    }
				}  
			  
				if(gain > maxGain){
				  gain = maxGain;
				}
			  
				if(gain < minGain){
				  gain = minGain;
				}
				
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
			
			virtual void enableSmoothChange(bool enable){
				smoothChange = enable;
			}
			
			virtual void setGain(Tgain c) {
				if(c <= maxGain && c >= minGain) {
				  if(smoothChange) {
				    targetGain = c;
				  }else {
				    gain = c;
				  }
				}
			}
			
			virtual void setMaxGain(Tgain maxGain) {
				this->maxGain = maxGain;
			}
			
			virtual void setMinGain(Tgain minGain) {
				this->minGain = minGain;
			}
			
			virtual void setGainDiff(Tgain gainDiff) {
				this->gainDiff = gainDiff;
			}

		protected:
			Tgain gain;
			Tgain maxGain;
			Tgain minGain;
			Tgain targetGain;
			Tgain gainDiff;
			bool enabled{true};
			bool smoothChange{false};
		};
		
				/********** Print functions **********/
		template <typename Tout, typename Tgain>
		std::ostream& operator<<(std::ostream& os, Gain<Tout,Tgain>& gain) {
			os << "Block gain: '" << gain.getName() << "' gain val = " << gain.gain; 
		}
	};
};

#endif /* ORG_EEROS_CONTROL_GAIN_HPP_ */
