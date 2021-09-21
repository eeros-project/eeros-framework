#ifndef ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP
#define ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
namespace control {
	
	/**
	* A low pass filter block is used to filter an input signal. 
	* The output signal value depends on the current and the past
	* input signal values, according to this equation:
	* valOut = valin*alpha + valprev*(1-alpha) 
	* 
	* LowPassFilter is a class template with one type template argument.
	* The type template argument specifies the type which is used for the 
	* values when the class template is instantiated.
	* 
	* If the LowPassFilter is used with matrices (Matrix, Vector), the filter algorithm
	* will consider all values in the matrice and will not separate them.
	* For example a 3-tuple of a Vector3 instance will be kept together during processing
	* in the LowPassFilter.\n
	* 
	* @tparam T - value type (double - default type)
	* 
	* @since TODO
	*/
	
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
		LowPassFilter(double alpha) : alpha(alpha) { 
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
			
			if (first) {
				valprev = valin;
				first = false;
			}
			
			if(enabled) {
				valOut = valin*alpha + valprev*(1-alpha); 
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
		
	protected:
		bool first{true};
		bool enabled{true};
		Signal<T> prev;
		T valOut;
	};
};
	
}

#endif /* ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP */
