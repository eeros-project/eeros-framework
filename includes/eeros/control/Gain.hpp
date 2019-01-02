#ifndef ORG_EEROS_CONTROL_GAIN_HPP_
#define ORG_EEROS_CONTROL_GAIN_HPP_

#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {

		/**
		 * A gain block is used to amplify an input signal. This is basically done by
		 * multiplying the gain set in the Gain instance with the input signal.
		 * The output signal is therefore the gain multiplied with the input signal.
		 * 
		 * output = gain * input
		 * 
		 * Gain is a Class Template with two type and one non-type template arguments.
		 * The two type template arguments specify the types which are used for the 
		 * output type and the gain type when the class template is instanciated.
		 * The non-type template argument specifies if the multiplication will be done
		 * element wise in case the gain is used with matrices.
		 * 
		 * @tparam Tout - output type (double - default type)
		 * @tparam Tgain - gain type (double - default type)
		 * @tparam elementWise - amplify element wise (false - default value)
		 * 
		 * @since v0.6
		 */
 
		template <typename Tout = double, typename Tgain = double, bool elementWise = false>
		class Gain : public Block1i1o<Tout> {
			
		public:
			/**
			 * Constructs a default gain instance with a gain of 1.\n
			 * Sets the max gain to 1000.\n
			 * Sets the min gain to -1000.\n
			 * Sets the target gain to 1.\n
			 * Sets the gain diff to 0.\n
			 */
			Gain() {
				gain = 1;
				maxGain = 1000;
				minGain = -1000;
				targetGain = gain;
				gainDiff = 0;
			}


			/**
			 * Constructs a default gain instance with a gain of the parameter c.\n
			 * Sets the max gain to 1000.\n
			 * Sets the min gain to -1000.\n
			 * Sets the target gain to the parameter c.\n
			 * Sets the gain diff to 0.\n
			 * @param c - initial gain value
			 */
			Gain(Tgain c) {
				gain = c; 
				maxGain = 1000;
				minGain = -1000;
				targetGain = gain;
				gainDiff = 0;
			}


			/**
			 * Constructs a gain instance with a gain of the parameter c,
			 * a maximum gain of maxGain and a minimum gain of minGain.\n
			 * Sets the target gain to the parameter c.\n
			 * Sets the gain diff to 0.\n
			 * @param c - initial gain value
			 * @param maxGain - initial maximum gain value
			 * @param minGain - initial minimum gain value
			 */
			Gain(Tgain c, Tgain maxGain, Tgain minGain = 1) {
				gain = c;
				this->maxGain = maxGain;
				this->minGain = minGain;
				targetGain = gain;
				gainDiff = 0;
			}


			/**
			 * Runs the amplification algorithm.
			 * 
			 * Performs the smooth change if smooth change is enabled with enableSmoothChange(bool).
			 * A smooth change of the gain is performed by adding or subtracting a gain differential
			 * specifiable by setGainDiff(Tgain).
			 * 
			 * Checks if gain is in the band in between minGain and maxGain or correct it otherwise.
			 * The correction is done by set the gain to maxGain or minGain respectively.
			 * 
			 * Sets the output signal to the amplified value calculated by multiplying gain * input if
			 * the gain instance is enabled by enable() and disable() respectively.
			 * 
			 * @see enableSmoothChange(bool)
			 * @see setGainDiff(Tgain)
			 * @see enable()
			 * @see disable()
			 */
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


			/**
			 * Enables the gain.
			 * 
			 * If enabled, run() will set the output signal to the amplified value calculated by multiplying gain * input
			 *
			 * Does not enable smooth change. This is done by calling enableSmoothChange(bool).
			 * 
			 * @see run()
			 * @see enableSmoothChange(bool) 
			 */
			virtual void enable() {
				enabled = true;
			}


			/**
			 * Disables the gain.
			 * 
			 * If disabled, run() will set the output signal to the input signal.
			 *
			 * Does not disable smooth change. This is done by calling enableSmoothChange(bool).
			 * 
			 * @see run()
			 * @see enableSmoothChange(bool) 
			 */
			virtual void disable() {
				enabled = false;
			}


			/**
			 * Enables or disables a smooth change of the gain.
			 * 
			 * If enabled, run() will perform a smooth change of the gain value.
			 *
			 * Does not enable or disable the gain. This is done by calling enable() and disable() respectively.
			 * 
			 * @param enable - enables or disables a smooth change of the gain
			 * 
			 * @see run()
			 * @see enable()
			 * @see disable() 
			 */
			virtual void enableSmoothChange(bool enable){
				smoothChange = enable;
			}


			/**
			 * Sets the gain value if smooth change is disabled and c is in the band in between minGain and maxGain.
			 *
			 * Sets the target gain value if smooth change is enabled and c is in the band in between minGain and maxGain.
			 * 
			 * Does not change gain or target gain value otherwise.
			 * 
			 * @param c - gain value
			 */
			virtual void setGain(Tgain c) {
				if(c <= maxGain && c >= minGain) {
				  if(smoothChange) {
				    targetGain = c;
				  }else {
				    gain = c;
				  }
				}
			}


			/**
			 * Sets the maximum allowed gain.
			 * @param maxGain - maximum allowed gain value
			 */
			virtual void setMaxGain(Tgain maxGain) {
				this->maxGain = maxGain;
			}


			/**
			 * Sets the minimum allowed gain.
			 * @param minGain - minimum allowed gain value
			 */
			virtual void setMinGain(Tgain minGain) {
				this->minGain = minGain;
			}


			/**
			 * Sets the gain differential needed by run() to perform a smooth gain change.
			 * @param gainDiff - gain differential
			 */
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


		/*
		 * The following class is a specialisation of Gain<Tout, Tgain, elementWise>.
		 * It is used if elements of a matrix must be amplified by the gain element wise.
		 * The most of the code is copied from the class template above. The only difference
		 * is the run() method.
		 * TODO: We need a solution so we do not have duplicated code.
		 */
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
		/**
		 * Enables writing a gain block to a std::ostream.
		 */
		template <typename Tout, typename Tgain>
		std::ostream& operator<<(std::ostream& os, Gain<Tout,Tgain>& gain) {
			os << "Block gain: '" << gain.getName() << "' gain val = " << gain.gain; 
		}
	};
};

#endif /* ORG_EEROS_CONTROL_GAIN_HPP_ */
