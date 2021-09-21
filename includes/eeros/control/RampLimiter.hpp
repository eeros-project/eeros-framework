#ifndef ORG_EEROS_CONTROL_RAMPLIMITER_HPP_
#define ORG_EEROS_CONTROL_RAMPLIMITER_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <math.h>


namespace eeros {
namespace control {
	/**
	* The Ramp Limiter block ... TODO
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
	
			if(enabled) {
				if(count < ramp_steps)
					count++;
				
				if(first) {
					old_val = inVal;
					start = inVal;
					first = false;
				}
				else if (fabs(inVal-old_val) > 0.002){  // new - old > 2mm
					incr_step = (inVal - old_val)/(ramp_steps+1);
					start = old_val + incr_step;
					count = 0;
					old_val = inVal;
					
					outVal = start;
				}
				else if (count >= ramp_steps) {
					outVal = inVal;
				}
				else {
					outVal = start + count * incr_step;
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
