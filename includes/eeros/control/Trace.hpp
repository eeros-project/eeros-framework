#ifndef ORG_EEROS_CONTROL_TRACE_HPP_
#define ORG_EEROS_CONTROL_TRACE_HPP_

#include <vector>
#include <eeros/control/Block1i.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Trace : public Block1i<T> {
		public:
			Trace(uint32_t bufLen) : maxBufLen(bufLen) {
				this->buf = new T[bufLen];
				this->timeBuf = new timestamp_t[bufLen];
			}

			virtual void run() {
				if (running) {
					buf[index] = this->in.getSignal().getValue();
					timeBuf[index] = this->in.getSignal().getTimestamp();
					index++;
					if (index == maxBufLen) {
						index = 0;
						cycle = true;
					}
				}
			}
			virtual T* getTrace() {
				if (cycle) {
					T* tmp = new T[maxBufLen];
					size = maxBufLen;
					for (int i = 0; i < maxBufLen; i++)
						tmp[i] = buf[(i + index) % maxBufLen];
					return tmp;
				} else {
					T* tmp = new T[index];
					size = index;
					for (int i = 0; i < index; i++)
						tmp[i] = buf[i];
					return tmp;
				}
			}
			virtual timestamp_t* getTimestampTrace() {
				if (cycle) {
					size = maxBufLen;
					timestamp_t* tmp = new timestamp_t[maxBufLen];
					for (int i = 0; i < maxBufLen; i++)
						tmp[i] = timeBuf[(i + index) % maxBufLen];
					return tmp;
				} else {
					timestamp_t* tmp = new timestamp_t[index];
					size = index;
					for (int i = 0; i < index; i++)
						tmp[i] = timeBuf[i];
					return tmp;
				}
			}
			virtual uint32_t getSize() {return size;}
			virtual void enable() {running = true;}
			virtual void disable() {running = false;}
			
		protected:
			uint32_t maxBufLen;	// total size of buffer
			uint32_t size;		// size to which the buffer is filled
			uint32_t index = 0;	// current index
			bool cycle = false;	// indicates whether wrap around occured
			bool running = false;	// indicates whether trace runs
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
