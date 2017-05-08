#ifndef ORG_EEROS_CONTROL_TRACE_HPP_
#define ORG_EEROS_CONTROL_TRACE_HPP_

#include <vector>
#include <eeros/control/Block1i.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Trace : public Block1i<T> {
		public:
			Trace(uint32_t bufLen) : bufLen(bufLen) {
				this->buf = new T[bufLen];
				this->timeBuf = new timestamp_t[bufLen];
			}

			virtual void run() {
				buf[current] = this->in.getSignal().getValue();
				timeBuf[current] = this->in.getSignal().getTimestamp();
				current = (current + 1) % bufLen;
			}
			virtual T* getTrace() {
				T* tmp = new T[bufLen];
				for (int i = 0; i < bufLen; i++)
					tmp[i] = buf[(i + current) % bufLen];
				return tmp;
			}
			virtual timestamp_t* getTimestampTrace() {
				timestamp_t* tmp = new timestamp_t[bufLen];
				for (int i = 0; i < bufLen; i++)
					tmp[i] = timeBuf[(i + current) % bufLen];
				return tmp;
			}
			
		protected:
			uint32_t bufLen;
			uint32_t current = 0;
			T* buf;
			timestamp_t* timeBuf;
		};
		
		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, Trace<T>& trace) {
			os << "Block trace: '" << trace.getName() << "'"; 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_TRACE_HPP_ */
