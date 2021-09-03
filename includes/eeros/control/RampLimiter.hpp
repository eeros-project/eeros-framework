#ifndef ORG_EEROS_CONTROL_RAMPLIMITER_HPP_
#define ORG_EEROS_CONTROL_RAMPLIMITER_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <math.h>


namespace eeros {
namespace control {
	/**
	* The Ramp Limiter block limits the first derivative of the signal passing through it. 
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
	
	template<typename Tout = double>
	class RampLimiter : public eeros::control::Blockio<Tout>
	{
	public:
		/**
		 * Constructs a RampLimiter instance. Needed delta_time and ramp_time in seconds \n
		 */
		RampLimiter(double dt_s, double ramp_time_s){
			ramp_steps = floor(ramp_time_s / dt_s - 0.5); 
		}
		
		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		RampLimiter(const RampLimiter& s) = delete; 
	
		/**
		 * Runs the Ramp limiter algorithm, as described above.
		 * 
		 * 
		 */
		virtual void run(){
			std::lock_guard<std::mutex> lock(mtx);
			Tout inVal = this->in.getSignal().getValue();
			
			Tout outVal = inVal;
			
// 			double input = this->in.getSignal().getValue();
// 			double output = input;
	
			if(enabled) {
				if(count < ramp_steps)
					count++;
				
				if(first) {
					old_val = input;
					start = input;
					first = false;
				}
				else if (fabs(input-old_val) > 0.002){  // new - old > 2mm
					incr_step = (input - old_val)/(ramp_steps+1);
					start = old_val + incr_step;
					count = 0;
					old_val = input;
					
					output = start;
				}
				else if (count >= ramp_steps) {
					output = input;
				}
				else {
					output = start + count * incr_step;
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
        
    private:
		double ramp_steps = 0;
		double count = 0;
		double incr_step = 0;
		double old_val = 0;
		double start = 0;
		
	protected:
		bool first{true};
		bool enabled{false};
		std::mutex mtx;
	};
}
}

#endif /* ORG_EEROS_CONTROL_RAMPLIMITER_HPP_ */
