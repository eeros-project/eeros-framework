#ifndef ORG_EEROS_CONTROL_MEDIANFILTER_HPP_
#define ORG_EEROS_CONTROL_MEDIANFILTER_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <algorithm>
#include <cmath>


namespace eeros {
	namespace control {

		/**
		 * A median filter (MedianFilter) block is used to filter an input signal. 
		 * The output signal value depends on the current and various past
		 * input signal values.
		 * The median filter algorithm sorts all values. The median value will be
		 * set as output signal value if the MedianFilter instance is enabled. 
		 * 
		 * MedianFilter is a class template with one type and one non-type template argument.
		 * The type template argument specifies the type which is used for the 
		 * values when the class template is instanciated.
		 * The non-type template argument specifies the number of concidered values.
		 * 
		 * If the MedianFilter is use with matrices (Matrix, Vector), the filter algorithm
		 * will concider all values in the matrice and will not separate them.
		 * For example a 3-tuple of a Vector3 instance will keep together during processing
		 * in the MedianFilter.\n
		 * If the sort algorithm can not sort the values, the will be kept in the same 
		 * order as when they were inserted into the MedianFilter.   
		 * 
		 * @tparam N - number of concidered values
		 * @tparam Tval - value type (double - default type)
		 * 
		 * @since v0.6
		 */

		template <size_t N, typename Tval = double>
		class MedianFilter : public Block1i1o<Tval> {

		public:
			/**
			 * Runs the filter algorithm.
			 * 
			 * Performs the calculation of the filtered output signal value.
			 * 
			 * Sorts the current and various past input signal values.
			 * The median value will be set as output signal value if 
			 * the MedianFilter instance is enabled. Otherwise, the output
			 * signal value is set to the actual input signal value.
			 * 
			 * The timestamp value will not be altered.
			 *
			 * @see enable()
			 * @see disable()
			 */
			virtual void run() {
				for(size_t i = 0; i < N; i++) {
					if(i < N-1) {
						currentValues[i] = currentValues[i+1];
					} else {
						currentValues[i] = this->in.getSignal().getValue();
					}
				}

				if(enabled) {
					Tval temp[N]{};

					std::copy(std::begin(currentValues), std::end(currentValues), std::begin(temp));
					std::sort(std::begin(temp), std::end(temp));

					currentMedianValue = temp[medianIndex];
					this->out.getSignal().setValue(currentMedianValue);

				} else {
					this->out.getSignal().setValue(this->in.getSignal().getValue());
				}

				this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
			}


			/**
			 * Enables the filter.
			 * 
			 * If enabled, run() will set the output signal value to the median value
			 * which results from sorting the current and the past values. 
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
			template <size_t No, typename ValT>
			friend std::ostream& operator<<(std::ostream& os, MedianFilter<No,ValT>& filter);


		protected:
			Tval currentValues[N]{};
			Tval currentMedianValue;
			bool enabled{true};
			constexpr static int medianIndex{static_cast<int>(floor(N/2))};
		};


		/**
		 * Operator overload (<<) to enable an easy way to print the state of a
		 * MedianFilter instance to an output stream.
		 * Does not print a newline control character.
		 */
		template <size_t N, typename Tval>
		std::ostream& operator<<(std::ostream& os, MedianFilter<N,Tval>& filter) {
			os << "Block MedianFilter: '" << filter.getName() << "' is enabled=";
			os << filter.enabled << ", ";

			os << "current median=" << filter.currentMedianValue << ", ";

			os << "medianIndex=" << filter.medianIndex << ", ";

			os << "current values:[" << filter.currentValues[0];
			for(size_t i = 1; i < N; i++){
				os << "," << filter.currentValues[i];
			}
			os << "]";
		}
	};
};

#endif /* ORG_EEROS_CONTROL_MEDIANFILTER_HPP_ */
