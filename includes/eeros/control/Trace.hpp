#ifndef ORG_EEROS_CONTROL_TRACE_HPP_
#define ORG_EEROS_CONTROL_TRACE_HPP_

#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <eeros/control/Block1i.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/logger/Logger.hpp>

#include <time.h>

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
					size = maxBufLen;
					T* tmp = new T[maxBufLen];
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
					size = index;
					timestamp_t* tmp = new timestamp_t[index];
					for (int i = 0; i < index; i++)
						tmp[i] = timeBuf[i];
					return tmp;
				}
			}
			virtual uint32_t getSize() {return size;}
			virtual void enable() {running = true;}
			virtual void disable() {running = false;}
			
			uint32_t maxBufLen;	// total size of buffer
			
		protected:
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


		template < typename T = double >
		class TraceWriter : public eeros::Thread {
		public:
			explicit TraceWriter(Trace<T>& trace, std::string fileName, int priority = 5) : Thread(priority), trace(trace), name(fileName) { }
			~TraceWriter() {running = false;}
			void write() {go = true;}
			
		private:
			bool running = false, go = false;
			virtual void run() {
				running = true;
				while(running) {
					while(running && !go) usleep(1000);
					if (!running) return;
					go = false;
					log.info() << "start writing trace file " + name;
					std::ofstream file;
					
					time_t now = time(0);
					struct tm  tstruct;
					char       chbuf[80];
					tstruct = *localtime(&now);
					strftime(chbuf, sizeof(chbuf), "_%Y-%m-%d_%X", &tstruct);
					
					file.open(name + chbuf, std::ios::trunc);
					timestamp_t* timeStampBuf = trace.getTimestampTrace();
					T* buf = trace.getTrace();
					file << "name = " << trace.getName() << ", size = " << trace.getSize() << ", maxBufLen = " << trace.maxBufLen << "\n";
					for (int i = 0; i < trace.getSize(); i++) file << timeStampBuf[i] << " " << buf[i] << std::endl;
					file.close();
					log.info() << "trace file written";
				}
			}
			std::string name;
			Trace<T>& trace;
			eeros::logger::Logger log;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_TRACE_HPP_ */
