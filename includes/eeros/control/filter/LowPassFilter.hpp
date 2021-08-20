#ifndef ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP
#define ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
namespace control {
	
	template<typename T = double>
	class LowPassFilter : public eeros::control::Blockio<1,1,T>
	{
	public:
		/**
		* Constructs a default low pass filter, with an alpha value as on parameter \n
		* Calls LowPassFilter(Tout alpha).
		*
		* @see LowPassFilter(Tout alpha)
		* @param alpha - weight of input in the filter (value between 0 and 1)
		*/
		LowPassFilter(T alpha) : alpha(alpha) { 
			prev.clear();
		}
			
		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		LowPassFilter(const LowPassFilter& s) = delete; 
		
		/**
		* Runs the filter algorithm.
		*
		* Reads input data
		* Calculates output depending on alpha param, input and previous output:
		* out = alpha*in + (1-alpha)*out_prev
		* Saves output for next run
		*/
		virtual void run(){
			Signal<T> sig = this->in.getSignal(); 
			T valin = sig.getValue();
			T valprev = prev.getValue();
			
			if(enabled) {
				valOut = valin*alpha+ valprev*(1-alpha); 
			}
			else {
				valOut = valin;
			}

			this->out.getSignal().setValue(valOut);
			this->out.getSignal().setTimestamp(sig.getTimestamp());
			
			prev = this->out.getSignal();
		}
		
		/**
		* Enables the gain.
		*
		* If enabled, run() will set the output signal to the filtered value
		*
		* @see run()
		*/
		virtual void enable() {
			enabled = true;
		}

		/**
		* Disables the gain.
		*
		* If disabled, run() will set the output signal to the input signal.
		*
		* @see run()
		*/
		virtual void disable() {
			enabled = false;
		}

	private:
		double alpha{1.0};
		bool enabled{true};
		Signal<T> prev;
		T valOut;
	};
};
	
}

#endif /* ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP */
