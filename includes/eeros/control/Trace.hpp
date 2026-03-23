#ifndef ORG_EEROS_CONTROL_TRACE_HPP_
#define ORG_EEROS_CONTROL_TRACE_HPP_

#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <eeros/control/Blockio.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/logger/Logger.hpp>

#include <time.h>

namespace eeros {
namespace control {

template < typename T = double >
class Trace : public Blockio<1,0,T> {
 public:
  Trace(uint32_t bufLen) : maxBufLen(bufLen) {
    buf = new T[bufLen];
    timeBuf = new timestamp_t[bufLen];
  }
  
  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  Trace(const Trace& s) = delete; 

  /**
   * Destructor frees buffers.
   */
  ~Trace() {
    delete[] buf;
    delete[] timeBuf;
  }

  virtual void run() override {
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
      for (uint32_t i = 0; i < maxBufLen; i++)
        tmp[i] = buf[(i + index) % maxBufLen];
      return tmp;
    } else {
      T* tmp = new T[index];
      size = index;
      for (uint32_t i = 0; i < index; i++)
        tmp[i] = buf[i];
      return tmp;
    }
  }
  virtual timestamp_t* getTimestampTrace() {
    if (cycle) {
      size = maxBufLen;
      timestamp_t* tmp = new timestamp_t[maxBufLen];
      for (uint32_t i = 0; i < maxBufLen; i++)
        tmp[i] = timeBuf[(i + index) % maxBufLen];
      return tmp;
    } else {
      size = index;
      timestamp_t* tmp = new timestamp_t[index];
      for (uint32_t i = 0; i < index; i++)
        tmp[i] = timeBuf[i];
      return tmp;
    }
  }
  virtual uint32_t getSize() {return size;}
  virtual void enable() override {running = true;}
  virtual void disable() override {running = false;}
  
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
  return os;
}


template < typename T = double >
class TraceWriter : public eeros::Thread {
public:
  explicit TraceWriter(Trace<T>& trace, std::string fileName, int priority = 20) 
      : Thread(priority), trace(trace), name(fileName), log(logger::Logger::getLogger()) { }
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
      for (uint32_t i = 0; i < trace.getSize(); i++) file << timeStampBuf[i] << " " << buf[i] << std::endl;
      file.close();
      log.info() << "trace file written";
    }
  }
  Trace<T>& trace;
  std::string name;
  logger::Logger log;
};

};
};

#endif /* ORG_EEROS_CONTROL_TRACE_HPP_ */
