#ifndef ORG_EEROS_CONTROL_WRAPAROUND_HPP_
#define ORG_EEROS_CONTROL_WRAPAROUND_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>


namespace eeros {
namespace control {
/**
* The WrapAround block wraps an input value between two limit values. 
* The output value will also vary between minVal and maxVal.
* 
* @tparam Tout - output type (double - default type) // TODO for double, at the moment with matrix
* @tparam minVal: minimum value - lower value of output signal
* @tparam maxVal: maximum value - higher value of output signal
*  
* @since xxx
*/
	
	template<typename Tout = eeros::math::Vector2>
	class WrapAround : public eeros::control::Blockio<1,1,Tout>
	{
		public:
			/**
			 * Constructs a WrapAround instance specifying minValue and maxValue of output 
			 * to realize wrap. \n
			 */
			WrapAround(double minVal, double maxVal) : enabled(true) {
				this->minVal = minVal;
				this->maxVal = maxVal;
			}
			
			/**
			 * Disabling use of copy constructor because the block should never be copied unintentionally.
			 */
			WrapAround(const WrapAround& s) = delete; 
	
			/**
			 * Runs the Wrap Around Algrithm, as described above.
			 * 
			 */
			virtual void run(){
				std::lock_guard<std::mutex> lock(mtx);
				Tout inVal = this->in.getSignal().getValue();
				Tout outVal = inVal;
				
				if(enabled) {
					for(int i=0; i<inVal.size(); i++) {
						double delta = fabs(this->minVal) + fabs(this->maxVal);
						
						double num = inVal[i] - this->minVal;
						double den = delta;
						double tquot = floor(num/den);
						outVal[i] = num - tquot * den;
						
						if(outVal[i]<0) {
							outVal[i] = outVal[i] + delta;
						}
						outVal[i] = outVal[i] + this->minVal; 
					}
				}
					
				this->out.getSignal().setValue(outVal);
				this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
			}
			
			/**
			 * Enables the rate limiter block.
			 * 
			 * If enabled, run() will perform rate limit.
			 * 
			 * @see disable()
			 */
			virtual void enable() {
				enabled = true;
			}
			
			/**
			 * Disables the rate limiter block.
			 * 
			 * If disabled, run() will set output = input.
			 * 
			 * @see enable()
			 */
			virtual void disable() {
				enabled = false;
			}
			
			/**
			* Sets min and max values.
			*/
			virtual void setMinMax(double minVal, double maxVal) {
				std::lock_guard<std::mutex> lock(mtx);
				this->minVal = minVal;
				this->maxVal = maxVal;
			}

		protected:
			bool enabled{true};
			std::mutex mtx;
			double minVal{0.0};
			double maxVal{0.0};
	};
}
}

#endif /* ORG_EEROS_CONTROL_WRAPAROUND_HPP_ */
