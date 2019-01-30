#ifndef ORG_EEROS_CONTROL_MAFILTER_HPP_
#define ORG_EEROS_CONTROL_MAFILTER_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <type_traits>
#include <ostream>


namespace eeros {
	namespace control {

		/**
		 * A moving average filter (MAFilter) block is used to filter an input signal. 
		 * The output signal value depends linearly on the current and various past
		 * input signal values.
		 * This is achieved by multiplying the current and the past values with
		 * the specified coefficients. The results are accumulated and lead to the
		 * output signal value. The following terms represents the operation performed in
		 * this block.
		 * 
		 * y[t] = c[0]*x[t-N]  + c[1]*x[t-N+1]  + ... +  c[N]*x[t]
		 *
		 * Outp = c[0]*prev[0] + c[1]*prev[1]   + ... +  c[N]*Inp
		 * 
		 * MAFilter is a class template with two type and one non-type template arguments.
		 * The two type template arguments specify the types which are used for the 
		 * values and the coefficients when the class template is instanciated.
		 * The non-type template argument specifies the number of coefficients and the
		 * number of concidered past values respectively.
		 * 
		 * @tparam N - number of coefficients
		 * @tparam Tval - value type (double - default type)
		 * @tparam Tcoeff - coefficients type (Tval - default value)
		 * 
		 * @since v0.6
		 */

		template <size_t N, typename Tval = double, typename Tcoeff = Tval>
		class MAFilter : public Block1i1o<Tval> {

		public:
			/**
			 * Constructs a MAFilter instance with the coefficients coeff.\n
			 * @param coeff - coefficients
			 */
			explicit MAFilter(Tcoeff (& coeff)[N]) : coefficients{coeff} {
				zeroInitPreviousValues<Tval>();
			}


			/**
			 * Runs the filter algorithm.
			 * 
			 * Performs the calculation of the filtered output signal value.
			 * Multiplies the current and past input signal values with the coefficients.
			 * The coefficients weight the current and past input signal values. Finally, 
			 * the resulting values are accumulated and yield to the output signal value
			 * if the filter instance is enabled. Otherwise, the output signal value is 
			 * set to the actual input signal value.
			 * 
			 * The timestamp value will not be altered.
			 *
			 * @see enable()
			 * @see disable()
			 */
			virtual void run() {
				 Tval actualValue = this->in.getSignal().getValue();
				Tval result = coefficients[N-1] * actualValue;

				for(size_t i = 0; i < N-1; i++) {
					previousValues[i] = previousValues[i+1];
					result += coefficients[i] * previousValues[i];
				}
				previousValues[N-1] = actualValue;

				if(enabled) {
					this->out.getSignal().setValue(result);
				} else {
					this->out.getSignal().setValue(this->in.getSignal().getValue());
				}

				this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
			}


			/**
			 * Enables the filter.
			 * 
			 * If enabled, run() will set the output signal value to the accumulated values
			 * which result from the current and the past values weighted by the coefficients. 
			 * 
			 * @see run()
			 */
			virtual void enable() {
				enabled = true;
			}


			/**
			 * Disables the filter.
			 * 
			 * If disabled, run() will set the output signal to the input signal.
			 *
			 * @see run()
			 */
			virtual void disable() {
				enabled = false;
			}


			/*
			 * Friend operator overload to give the operator overload outside 
			 * the class access to the private fields.
			 */
			template <size_t No, typename ValT, typename CoeffT>
			friend std::ostream& operator<<(std::ostream& os, MAFilter<No,ValT,CoeffT>& filter);


		protected:
			Tcoeff * coefficients;
			Tval previousValues[N]{};
			bool enabled{true};


		private:
			template <typename S>
			typename std::enable_if<std::is_arithmetic<S>::value>::type zeroInitPreviousValues() {
				// is zeroed when initialized by default.
			}


			template <typename S>
			typename std::enable_if<!std::is_arithmetic<S>::value>::type zeroInitPreviousValues() {
				  for(size_t i = 0; i < N; i++) {
					previousValues[i].zero();
				  }
			}
		};


		/**
		 * Operator overload (<<) to enable an easy way to print the state of a
		 * MAFilter instance to an output stream.
		 * Does not print a newline control character.
		 */
		template <size_t N, typename Tval, typename Tcoeff>
		std::ostream& operator<<(std::ostream& os, MAFilter<N,Tval,Tcoeff>& filter) {
			os << "Block MAFilter: '" << filter.getName() << "' is enabled=";
			os << filter.enabled << ", ";

			os << "coefficients:[" << filter.coefficients[0];
			for(size_t i = 1; i < N; i++){
				os << "," << filter.coefficients[i];
			}
			os << "], ";

			os << "previousValues:[" << filter.previousValues[0];
			for(size_t i = 1; i < N; i++){
				os << "," << filter.previousValues[i];
			}
			os << "]";
		}
	};
};

#endif /* ORG_EEROS_CONTROL_MAFILTER_HPP_ */
