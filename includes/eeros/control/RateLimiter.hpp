#ifndef ORG_EEROS_CONTROL_RATELIMITER_HPP_
#define ORG_EEROS_CONTROL_RATELIMITER_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>


namespace eeros {
namespace control {
	
/**
 * The Rate Limiter block limits the first derivative of the signal passing through it. 
 * The output changes no faster than the specified limit. 
 * The derivative is calculated using this equation: 
 * 		rate = (input(i)-output(i-1))/(time(i)-time(i-1))
 * 
 * The output is determined by comparing rate to the rising slew rate and falling slew rate parameters:
 * If rate is greater than the Rising slew rate parameter, the output is calculated as:
 * 		output(i) = dt * rising_slew_rate + output(i-1)
 * If rate is less than the Falling slew rate parameter, the output is calculated as:
 * 		output(i) = dt * falling_slew_rate + output(i-1)
 * If rate is between the bounds of R and F, the change in output is equal to the change in input: 
 * 		output(i) = input(i)
 *
 * If the input is a vector, then it is possible to choose if the algorithm applies element wise or not.
 * If yes, rising_slew_rate and falling_slew_rate must be vectors. 
 * 
 * @tparam Tout - output type (double - default type)
 * @tparam Trate - rate type (double - default type)
 * @tparam elementWise - apply element wise (false - default value). If true, then rates must be double.
 * @tparam rising_slew_rate: the limit of the derivative of an increasing input signal.
 * @tparam falling_slew_rate: the limit of the derivative of a decreasing input signal (negative value).
 *
 * @since xxx
 */
		
	template<typename Tout = double, typename Trate = double, bool elementWise = false>
	class RateLimiter : public Blockio<1,1,Tout> 
	{
	public:
		/**
		 * Constructs a RateLimiter instance with rising and falling rates equal \n
		 */
		RateLimiter(Trate rate) {
			falling_slew_rate = -rate;
			rising_slew_rate = rate;
			
			outPrev.clear();
		}
		
		/**
		 * Constructs a RateLimiter instance specifying rising and falling \n
		 */
		RateLimiter(Trate falling_rate, Trate rising_rate) {
			falling_slew_rate = falling_rate;
			rising_slew_rate = rising_rate;
			
			outPrev.clear();
		}
		
		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		RateLimiter(const RateLimiter& s) = delete; 
	
		/**
		 * Runs the Rate limiter algorithm, as described above.
		 * 
		 * rate = (input(i)-output(i-1))/(time(i)-time(i-1))
		 * 
		 * If rate > rising_slew_rate: output(i) = dt * rising_slew_rate + output(i-1)
		 * If rate < falling_slew_rate: output(i) = dt * falling_slew_rate + output(i-1)
		 * Otherwise: output(i) = input(i)
		 * 
		 */
		virtual void run(){
			std::lock_guard<std::mutex> lock(mtx);
			Tout inVal = this->in.getSignal().getValue();
			double tin = this->in.getSignal().getTimestamp() / 1000000000.0;
			double tprev = outPrev.getTimestamp() / 1000000000.0;
			
			Tout outVal = inVal;
			
			if(enabled) {
				double dt = tin - tprev;
				Trate rate;
				
				if(std::is_same<double, Tout>::value){
					rate = (inVal-outPrev.getValue())/dt;
					
					if(rate > rising_slew_rate) {
						outVal = dt * rising_slew_rate + outPrev.getValue();
					}
					else if(rate < falling_slew_rate) {
						outVal = dt * falling_slew_rate + outPrev.getValue();
					}
				
				}
// 				else {
// 					for(unsigned int i = 0; i < inVal.size(); i++) {
// 						rate[i] = (inVal[i]-outPrev.getValue()[i])/dt;
// 						
// 						if(!elementWise) {
// 							if(rate[i] > rising_slew_rate) {
// 								outVal[i] = dt * rising_slew_rate + outPrev.getValue()[i];
// 							}
// 							else if(rate[i] < falling_slew_rate) {
// 								outVal[i] = dt * falling_slew_rate + outPrev.getValue()[i];
// 							}
// 						}
// 						else {
// 							if(rate[i] > rising_slew_rate[i]) {
// 								outVal[i] = dt * rising_slew_rate[i] + outPrev.getValue()[i];
// 							}
// 							else if(rate[i] < falling_slew_rate[i]) {
// 								outVal[i] = dt * falling_slew_rate[i] + outPrev.getValue()[i];
// 							}
// 						}
// 					}
// 				}
				
				// this->out.getSignal().setValue(calculateResults<Tout>(this->in.getSignal().getValue()));
			}
			outPrev.setValue(outVal);
			outPrev.setTimestamp(this->in.getSignal().getTimestamp());
			
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
		 * Sets falling and rising rates.
		 */
		virtual void setRate(Trate falling_rate, Trate rising_rate) {
			std::lock_guard<std::mutex> lock(mtx);
			falling_slew_rate = falling_rate;
			rising_slew_rate = rising_rate;
		}
		
	private:
		Signal<Tout> outPrev;
		
// 		template<typename S>
// 		typename std::enable_if<!elementWise, S>::type calculateRateOutput(S value) {
// 			return gain * value;
// 		}
// 
// 
// 		template<typename S>
// 		typename std::enable_if<elementWise, S>::type calculateResults(S value) {
// 			return value.multiplyElementWise(gain);
// 		}
		
	protected:
		Trate falling_slew_rate, rising_slew_rate;
		bool enabled{false};
		std::mutex mtx;
	};
}
}

#endif /* ORG_EEROS_CONTROL_RATELIMITER_HPP_ */